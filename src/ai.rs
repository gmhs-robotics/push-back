use core::f64::consts::PI;

use pid::Pid;
use vexide::{devices::smart::GpsSensor, prelude::*};

use crate::{MAX_AUTO, Robot, WHEEL_DIAMETER};

pub trait Ai {
    async fn rotate_to(&mut self, deg: f64);
    async fn drive_by(&mut self, distance: f64);
    // async fn drive_to(&mut self, point: Point2<f64>);
}

impl Ai for Robot {
    #[cfg(true)]
    async fn drive_by(&mut self, distance: f64) {
        use core::time::Duration;

        const RPM: i32 = 120;

        // Calculate the number of wheel revolutions needed to cover the distance
        let revolutions = distance / (PI * WHEEL_DIAMETER);
        let target = MotorControl::Position(Position::from_revolutions(revolutions), RPM);

        // Reset motor encoders to ensure consistent starting positions
        self.left_front.reset_position().ok();
        self.left_back.reset_position().ok();
        self.right_front.reset_position().ok();
        self.right_back.reset_position().ok();

        // Set movement targets for all drive motors
        self.left_front.set_target(target).ok();
        self.left_back.set_target(target).ok();
        self.right_front.set_target(target).ok();
        self.right_back.set_target(target).ok();

        sleep(Duration::from_secs_f64(
            revolutions.abs() / (RPM as f64) * 60.,
        ))
        .await;
    }

    #[cfg(false)]
    async fn drive_by(&mut self, distance: f64) {
        // Tunables
        const MAX_V: f64 = MAX_AUTO;
        const MIN_V: f64 = 6.0;
        const KP_DIST: f64 = 8.0; // volts per meter
        const SLOW_RADIUS_M: f64 = 0.40;
        const ARRIVE_TOLERANCE_M: f64 = 0.05;
        let poll_interval = GpsSensor::UPDATE_INTERVAL;

        // Simple yaw PID gains for heading hold (tune as needed)
        const KP_YAW: f64 = 1.2;
        const KD_YAW: f64 = 0.12;

        // 1) Block until we get a valid start position+heading (no startup timeout)
        let (start_pos, start_heading_deg) = loop {
            match (self.gps.position(), self.gps.heading()) {
                (Ok(p), Ok(h)) => break (p, h.rem_euclid(360.0)),
                _ => sleep(poll_interval).await,
            }
        };

        // 2) Goal point computed along start heading
        let heading_rad = start_heading_deg.to_radians();
        let goal_x = start_pos.x + distance * heading_rad.cos();
        let goal_y = start_pos.y + distance * heading_rad.sin();

        // 3) Yaw PID (setpoint = 0; we'll pass yaw_error as measurement)
        let mut yaw_pid = Pid::new(0.0, MAX_V);
        yaw_pid.p(KP_YAW, MAX_V);
        yaw_pid.d(KD_YAW, MAX_V);

        // 4) Drive loop
        loop {
            // Read GPS
            let (pos, heading_deg) = match (self.gps.position(), self.gps.heading()) {
                (Ok(p), Ok(h)) => (p, h.rem_euclid(360.0)),
                _ => {
                    sleep(poll_interval).await;
                    continue;
                }
            };

            // Remaining distance
            let dx = goal_x - pos.x;
            let dy = goal_y - pos.y;
            let remaining = (dx * dx + dy * dy).sqrt();

            // Arrival check
            if remaining <= ARRIVE_TOLERANCE_M {
                break;
            }

            // Forward voltage from distance (proportional, slow near target)
            let mut v = KP_DIST * remaining;
            if remaining < SLOW_RADIUS_M {
                v *= remaining / SLOW_RADIUS_M;
            }

            // Clamp forward and enforce minimum (to overcome stiction)
            if v.abs() > MAX_V {
                v = v.signum() * MAX_V;
            }
            if v.abs() < MIN_V {
                v = v.signum() * MIN_V;
            }
            let v = v.clamp(-MAX_V, MAX_V);

            // Heading hold: error between start heading and current
            let yaw_error = shortest_angle_error_deg(start_heading_deg, heading_deg);
            // pid computes (setpoint - measurement) so with setpoint=0, raw output ~= -K*error
            let pid_out = yaw_pid.next_control_output(yaw_error);
            let rot = -pid_out.output; // rot positive -> turn left, consistent with mapping below

            // Mecanum forward + yaw mapping (simple differential for yaw)
            let mut lf = v + rot;
            let mut lb = v + rot;
            let mut rf = v - rot;
            let mut rb = v - rot;

            // Per-wheel clamp
            lf = lf.clamp(-MAX_V, MAX_V);
            lb = lb.clamp(-MAX_V, MAX_V);
            rf = rf.clamp(-MAX_V, MAX_V);
            rb = rb.clamp(-MAX_V, MAX_V);

            // Apply minimum threshold only when command is non-zero
            let apply_min = |val: f64| -> f64 {
                if val.abs() > 0.0 && val.abs() < MIN_V {
                    val.signum() * MIN_V
                } else {
                    val
                }
            };
            lf = apply_min(lf).clamp(-MAX_V, MAX_V);
            lb = apply_min(lb).clamp(-MAX_V, MAX_V);
            rf = apply_min(rf).clamp(-MAX_V, MAX_V);
            rb = apply_min(rb).clamp(-MAX_V, MAX_V);

            // Command motors
            let _ = self.left_front.set_voltage(lf);
            let _ = self.left_back.set_voltage(lb);
            let _ = self.right_front.set_voltage(rf);
            let _ = self.right_back.set_voltage(rb);

            // sleep until next sample
            sleep(poll_interval).await;
        }

        // Final stop
        let _ = self.left_front.set_voltage(0.);
        let _ = self.left_back.set_voltage(0.);
        let _ = self.right_front.set_voltage(0.);
        let _ = self.right_back.set_voltage(0.);
    }

    #[cfg(false)]
    async fn drive_by(&mut self, distance: f64) {
        use vexide::time::Instant;

        // Tunables
        const MAX_V: f64 = MAX_AUTO;
        const MIN_V: f64 = 6.0;
        const KP_DIST: f64 = 8.0; // volts per meter (tune to your robot)
        const SLOW_RADIUS_M: f64 = 0.40; // start slowing inside this radius
        const ARRIVE_TOLERANCE_M: f64 = 0.05; // 5 cm arrival tolerance
        const MAX_TIMEOUT_SECS: f64 = 30.0; // safety timeout
        let poll_interval = GpsSensor::UPDATE_INTERVAL;

        // 1) Read starting GPS position + heading once (used to compute the goal point)
        let (start_pos, start_heading_deg) = loop {
            match (self.gps.position(), self.gps.heading()) {
                (Ok(p), Ok(h)) => break (p, h.rem_euclid(360.0)),
                _ => sleep(poll_interval).await,
            }
        };

        // 2) Compute goal point once (straight ahead from start along heading)
        let heading_rad = start_heading_deg.to_radians();
        let goal_x = start_pos.x + distance * heading_rad.cos();
        let goal_y = start_pos.y + distance * heading_rad.sin();

        // 3) (Optional) If you want encoder feedforward, set position targets here.
        //    Omitted for simplicity since you asked to rely on GPS for correction.

        // 4) GPS-supervised drive loop (no angle/heading corrections while driving)
        let start_time = Instant::now();
        loop {
            // Safety timeout
            if start_time.elapsed().as_secs_f64() > MAX_TIMEOUT_SECS {
                break;
            }

            // Read GPS position
            let pos = match self.gps.position() {
                Ok(p) => p,
                Err(_) => {
                    sleep(poll_interval).await;
                    continue;
                }
            };

            // Remaining distance to goal
            let dx = goal_x - pos.x;
            let dy = goal_y - pos.y;
            let remaining = (dx * dx + dy * dy).sqrt();

            // If close enough, stop
            if remaining <= ARRIVE_TOLERANCE_M {
                break;
            }

            // Map remaining distance -> forward voltage (proportional), slow when near
            let mut v = KP_DIST * remaining;
            if remaining < SLOW_RADIUS_M {
                v *= remaining / SLOW_RADIUS_M;
            }

            // Clamp and enforce minimum to overcome stiction
            if v.abs() > MAX_V {
                v = v.signum() * MAX_V;
            }
            if v.abs() < MIN_V {
                v = v.signum() * MIN_V;
            }

            let v = v.clamp(-MAX_V, MAX_V);

            // Apply same voltage to both sides (no steering/angle adjustments)
            self.left_front.set_voltage(v).ok();
            self.left_back.set_voltage(v).ok();
            self.right_front.set_voltage(v).ok();
            self.right_back.set_voltage(v).ok();

            sleep(poll_interval).await;
        }

        // Final stop
        self.left_front.set_voltage(0.).ok();
        self.left_back.set_voltage(0.).ok();
        self.right_front.set_voltage(0.).ok();
        self.right_back.set_voltage(0.).ok();
    }

    #[cfg(true)]
    async fn rotate_to(&mut self, deg: f64) {
        const KP: f64 = 1.2; // P gain (tune down if it oscillates)
        const MAX_V: f64 = MAX_AUTO; // maximum motor voltage
        const MIN_V: f64 = 6.0; // minimum effective voltage to overcome stiction
        const DEAD_BAND_DEG: f64 = 10.0; // Stop tolerance
        const SLOW_DOWN_ANGLE: f64 = 30.0; // angle (deg) within which we scale down speed

        loop {
            let rotation = match self.gps.heading() {
                Ok(r) => r,
                Err(_) => {
                    sleep(GpsSensor::UPDATE_INTERVAL).await;
                    continue;
                }
            };

            // shortest angular difference in range (-180, +180]
            let error = (deg - rotation + 540.0).rem_euclid(360.0) - 180.0;

            // Stop if we're within deadband
            if error.abs() <= DEAD_BAND_DEG {
                break;
            }

            // Proportional control:
            // Map error (-180..180) -> voltage (-MAX_V..MAX_V) and scale with KP
            let mut voltage = (error / 180.0) * MAX_V * KP;

            // When we're inside SLOW_DOWN_ANGLE, scale voltage down linearly so we don't overshoot
            if error.abs() < SLOW_DOWN_ANGLE {
                let scale = error.abs() / SLOW_DOWN_ANGLE; // 0..1
                voltage *= scale;
            }

            // Ensure we have at least a small voltage to overcome stiction (but don't exceed MAX_V)
            if voltage.abs() < MIN_V {
                voltage = error.signum() * MIN_V;
            }

            voltage = voltage.clamp(-MAX_V, MAX_V);

            self.left_front.set_voltage(voltage).ok();
            self.left_back.set_voltage(voltage).ok();
            self.right_front.set_voltage(-voltage).ok();
            self.right_back.set_voltage(-voltage).ok();

            sleep(GpsSensor::UPDATE_INTERVAL).await;
        }

        self.left_front.set_voltage(0.).ok();
        self.left_back.set_voltage(0.).ok();
        self.right_front.set_voltage(0.).ok();
        self.right_back.set_voltage(0.).ok();
    }

    #[cfg(false)]
    async fn rotate_to(&mut self, deg: f64) {
        // PID-based rotation controller (uses angular shortest-path error)
        const MAX_V: f64 = MAX_AUTO;
        const MIN_V: f64 = 6.0;
        const DEAD_BAND_DEG: f64 = 3.0; // tighter tolerance if you want
        // You can scale this larger if your GPS/heading is noisy.
        // Yaw PID gains (start with P; add a little D if it oscillates)
        const KP_YAW: f64 = 1.2; // tune
        const KD_YAW: f64 = 0.18; // tune

        // PID: setpoint = 0.0; we pass the signed angular error as the "measurement"
        let mut yaw_pid = Pid::new(0.0, MAX_V);
        yaw_pid.p(KP_YAW, MAX_V);
        yaw_pid.d(KD_YAW, MAX_V);

        loop {
            let rotation = match self.gps.heading() {
                Ok(r) => r.rem_euclid(360.0),
                Err(_) => {
                    sleep(GpsSensor::UPDATE_INTERVAL).await;
                    continue;
                }
            };

            // shortest angular difference to target (deg)
            let error = shortest_angle_error_deg(deg.rem_euclid(360.0), rotation);

            // Stop if we're within deadband
            if error.abs() <= DEAD_BAND_DEG {
                break;
            }

            // Feed error to pid: yaw_pid setpoint = 0, measurement = error.
            // pid output = (setpoint - measurement) * gains = -error * gains.
            // We invert to get a control that is proportional to error (same sign as earlier impl).
            let pid_out = yaw_pid.next_control_output(error);
            let mut voltage = -pid_out.output;

            // Ensure we have at least a small voltage to overcome stiction
            if voltage.abs() < MIN_V {
                voltage = voltage.signum() * MIN_V;
            }
            voltage = voltage.clamp(-MAX_V, MAX_V);

            // Mecanum rotation mapping: left motors positive, right motors negative for positive voltage
            self.left_front.set_voltage(voltage).ok();
            self.left_back.set_voltage(voltage).ok();
            self.right_front.set_voltage(-voltage).ok();
            self.right_back.set_voltage(-voltage).ok();

            sleep(GpsSensor::UPDATE_INTERVAL).await;
        }

        // Final stop
        self.left_front.set_voltage(0.).ok();
        self.left_back.set_voltage(0.).ok();
        self.right_front.set_voltage(0.).ok();
        self.right_back.set_voltage(0.).ok();
    }
}

fn shortest_angle_error_deg(target: f64, current: f64) -> f64 {
    (target - current + 540.0).rem_euclid(360.0) - 180.0
}
