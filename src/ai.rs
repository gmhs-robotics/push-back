use core::f64::{self, consts::PI};

use vexide::{devices::smart::GpsSensor, prelude::*, time::Instant};

use crate::{MAX_AUTO, Robot, WHEEL_DIAMETER};

pub trait Ai {
    async fn rotate_to(&mut self, deg: f64);
    async fn drive_by(&mut self, distance: f64);
    // async fn drive_to(&mut self, point: Point2<f64>);
}

impl Ai for Robot {
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

    /*    async fn drive_by(&mut self, distance: f64) {
        const RPM: i32 = 120;

        let revolutions = distance / (PI * WHEEL_DIAMETER);
        let target = MotorControl::Position(Position::from_revolutions(revolutions), RPM);

        // Could fetch the position of each motor and create their own drive target offset by the
        // position we got, but this is more efficient and I do not see any issues with
        // reset_position.
        self.left_front.reset_position().ok();
        self.left_back.reset_position().ok();
        self.right_front.reset_position().ok();
        self.right_back.reset_position().ok();

        self.left_front.set_target(target).ok();
        self.left_back.set_target(target).ok();
        self.right_front.set_target(target).ok();
        self.right_back.set_target(target).ok();
    }*/

    async fn drive_by(&mut self, distance: f64) {
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

    /* async fn drive_to(&mut self, goal: Point2<f64>) {
        const TOLERANCE: f64 = 0.05; // 5 cm
        const SLOW_RADIUS: f64 = 0.3; // slow down inside of 30 cm from goal
        const HEADING_CORRECT_DEG: f64 = 10.0;
        const RPM: i32 = 120;


        // Get a valid position - WAIT
        let position = loop {
            if let Ok(p) = self.gps.position() {
                break p;
            }
            sleep(GpsSensor::UPDATE_INTERVAL).await;
        };

        // Calculate initial goal and begin travelling to it
        {
            let dx = goal.x - position.x;
            let dy = goal.y - position.y;
            let dist = (dx * dx - dy * dy).sqrt();

            if dist <= TOLERANCE {
                self.left_front.set_voltage(0.).ok();
                self.left_back.set_voltage(0.).ok();
                self.right_front.set_voltage(0.).ok();
                self.right_back.set_voltage(0.).ok();

                return;
            }

            let heading = dy.atan2(dx).to_degrees().rem_euclid(360.0);

            self.rotate_to(heading).await;

            let revolutions = dist / (PI * WHEEL_DIAMETER);
            let target = MotorControl::Position(Position::from_revolutions(revolutions), RPM);

            // Could fetch the position of each motor and create their own drive target offset by the
            // position we got, but this is more efficient and I do not see any issues with
            // reset_position.
            self.left_front.reset_position().ok();
            self.left_back.reset_position().ok();
            self.right_front.reset_position().ok();
            self.right_back.reset_position().ok();

            self.left_front.set_target(target).ok();
            self.left_back.set_target(target).ok();
            self.right_front.set_target(target).ok();
            self.right_back.set_target(target).ok();
        }

        // Recalculate goal based on position every iteration
        {
            let start = Instant::now();

            loop {
                if let Ok(position) = self.gps.position() {
                    let dx = goal.x - position.x;
                    let dy = goal.y - position.y;
                    let dist = (dx * dx + dy * dy).sqrt();

                    // Here
                    if dist <= TOLERANCE {
                        break;
                    }

                    // Correct heading
                    if let Ok(heading) = self.gps.heading() {

                    }
                    if d < SLOW_RADIUS {
                        lety speed
                    }
                }

                sleep(GpsSensor::UPDATE_INTERVAL).await;
            }
        }

        // Assuming goal reached, stop all motors and end function
        self.left_front.set_voltage(0.).ok();
        self.left_back.set_voltage(0.).ok();
        self.right_front.set_voltage(0.).ok();
        self.right_back.set_voltage(0.).ok();
    }*/
}
