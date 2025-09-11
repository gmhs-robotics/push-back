//! Coordinate system for Robot

use core::f64::consts::PI;

use nalgebra::Vector3;
use vexide::float::Float;

// Wheel radius in meters
const WHEEL_RADIUS: f64 = 0.00;

/// encoder ticks per full wheel revolution
const TICKS_PER_REVOLUTION: f64 = 0.00;

/// distance between left and right wheels (track width) in meters
const TRACK_WIDTH: f64 = 0.00;

/// distance between front and back wheels (wheelbase) in meters
const WHEEL_BASE: f64 = 0.00;

// convert ticks -> linear wheel displacement (meters)
const DIST_PER_TICK: f64 = 2.0 * PI * WHEEL_RADIUS / TICKS_PER_REVOLUTION;

/// Mecanum odometry for 4-wheel X-configuration.
/// All distances are in meters and angles in radians/internal, but the returned Vector3
/// uses (X_meters, Y_meters, Theta_degrees_0_to_360).
pub struct MecanumOdometry {
    /// previous cumulative ticks (as read from encoders). Order: lf, rf, lb, rb
    prev_ticks: [i32; 4],

    /// current pose: x (m), y (m), theta (degrees 0..360)
    pose: Vector3<f64>,
}

impl MecanumOdometry {
    /// Create new odometry object.
    ///
    /// - initial_ticks: initial cumulative ticks for (lf, rf, lb, rb)
    /// - initial_pose: initial (x_m, y_m, theta_deg) — theta in degrees 0..360
    pub fn new(initial_ticks: [i32; 4], initial_pose: Vector3<f64>) -> Self {
        Self {
            prev_ticks: initial_ticks,
            pose: initial_pose,
        }
    }

    /// Update odometry with new cumulative encoder ticks and return updated pose (X, Y, Theta_deg).
    ///
    /// Input tick counts must be cumulative encoder positions (not deltas).
    /// Order: lf, rf, lb, rb.
    ///
    /// NOTE: If one or more encoders increase/decrease in the opposite sign relative to
    /// the physical forward direction, invert those encoder values (multiply by -1) before calling.
    pub fn update(
        &mut self,
        ticks_fl: i32,
        ticks_fr: i32,
        ticks_bl: i32,
        ticks_br: i32,
    ) -> Vector3<f64> {
        // 1) compute tick deltas (current cumulative minus previous cumulative)
        let dt_fl = ticks_fl - self.prev_ticks[0];
        let dt_fr = ticks_fr - self.prev_ticks[1];
        let dt_bl = ticks_bl - self.prev_ticks[2];
        let dt_br = ticks_br - self.prev_ticks[3];

        // 2) store for next iteration
        self.prev_ticks = [ticks_fl, ticks_fr, ticks_bl, ticks_br];

        let d_fl = (dt_fl as f64) * DIST_PER_TICK;
        let d_fr = (dt_fr as f64) * DIST_PER_TICK;
        let d_bl = (dt_bl as f64) * DIST_PER_TICK;
        let d_br = (dt_br as f64) * DIST_PER_TICK;

        // 3) robot-frame displacements (X forward, Y to robot's right)
        // Derived from forward kinematics for X-config 4-wheel mecanum:
        let dx_robot = (d_fl + d_fr + d_bl + d_br) / 4.0;
        let dy_robot = (-d_fl + d_fr + d_bl - d_br) / 4.0;

        // 4) rotation delta (radians)
        // R = l_x + l_y ; with wheel_base and track_width given, R = (wheel_base + track_width)/2
        let r = (WHEEL_BASE + TRACK_WIDTH) / 2.0;

        // avoid division-by-zero
        let dtheta = if r.abs() < 1e-12 {
            0.0
        } else {
            (-d_fl + d_fr - d_bl + d_br) / (4.0 * r)
        }; // radians

        // 5) rotate robot-frame displacement into global frame using previous heading
        let theta_prev_rad = self.pose[2].to_radians();
        let cos_t = theta_prev_rad.cos();
        let sin_t = theta_prev_rad.sin();
        let dx_global = cos_t * dx_robot - sin_t * dy_robot;
        let dy_global = sin_t * dx_robot + cos_t * dy_robot;

        // 6) accumulate pose
        self.pose[0] += dx_global;
        self.pose[1] += dy_global;

        let theta_new_rad = theta_prev_rad + dtheta;
        let mut theta_deg = theta_new_rad.to_degrees();

        // normalize to 0..360
        theta_deg = (theta_deg % 360.0 + 360.0) % 360.0;
        self.pose[2] = theta_deg;

        self.pose
    }

    /// Get current pose without updating
    pub fn get_pose(&self) -> Vector3<f64> {
        self.pose
    }
}
