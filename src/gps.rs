use evian::prelude::*;
use vexide::{
    devices::{
        math::{Point2, Vector3},
        smart::GpsSensor,
    },
    float::Float,
    time::Instant,
};

const VEC3_ZEROS: Vector3<f64> = Vector3 {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

pub struct GpsTracking {
    gps: GpsSensor,
    velocity: Vector3<f64>,
    last_update: Option<Instant>,
}

impl GpsTracking {
    pub fn new(gps: GpsSensor) -> Self {
        Self {
            gps,
            velocity: VEC3_ZEROS,
            last_update: None,
        }
    }

    pub fn integrate_acceleration(&mut self) {
        let accel = self.gps.acceleration().unwrap_or(VEC3_ZEROS);

        let now = Instant::now();

        if let Some(prev) = self.last_update {
            let dt = now.duration_since(prev).as_secs_f64();

            self.velocity.x += accel.x * dt;
            self.velocity.y += accel.y * dt;
            self.velocity.z += accel.z * dt;
        }

        self.last_update = Some(now);
    }
}

use evian::tracking::Tracking;

impl Tracking for GpsTracking {}

impl TracksPosition for GpsTracking {
    fn position(&self) -> Vec2<f64> {
        let pos = self.gps.position().unwrap_or(Point2 { x: 0., y: 0. });

        Vec2::new(pos.x, pos.y)
    }
}

impl TracksHeading for GpsTracking {
    fn heading(&self) -> Angle {
        let heading = self.gps.heading().unwrap_or(0.);

        Angle::from_degrees(heading)
    }
}

impl TracksVelocity for GpsTracking {
    fn linear_velocity(&self) -> f64 {
        (self.velocity.x * self.velocity.x
            + self.velocity.y * self.velocity.y
            + self.velocity.z * self.velocity.z)
            .sqrt()
    }

    fn angular_velocity(&self) -> f64 {
        self.gps.gyro_rate().map(|rate| rate.z).unwrap_or_default()
    }
}

impl TracksForwardTravel for GpsTracking {
    fn forward_travel(&self) -> f64 {
        0.
    }
}
