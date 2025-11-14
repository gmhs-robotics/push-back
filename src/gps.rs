use evian::{prelude::*, tracking::Tracking};
use vexide::{
    devices::{
        math::{Point2, Vector3},
        position,
        smart::GpsSensor,
    },
    float::Float,
};

pub struct GpsTracking(pub GpsSensor);

impl Tracking for GpsTracking {}

impl TracksPosition for GpsTracking {
    fn position(&self) -> Vec2<f64> {
        let pos = self.0.position().unwrap_or(Point2 { x: 0., y: 0. });

        Vec2::new(pos.x, pos.y)
    }
}

impl TracksHeading for GpsTracking {
    fn heading(&self) -> Angle {
        let heading = self.0.heading().unwrap_or(0.);

        Angle::from_degrees(heading)
    }
}

impl TracksVelocity for GpsTracking {
    fn linear_velocity(&self) -> f64 {
        let accel = self.0.acceleration().unwrap_or(Vector3 {
            x: 0.,
            y: 0.,
            z: 0.,
        });

        (accel.x * accel.x + accel.y * accel.y + accel.z * accel.z).sqrt()
    }

    fn angular_velocity(&self) -> f64 {
        let gyro = self.0.gyro_rate().unwrap_or(Vector3 {
            x: 0.,
            y: 0.,
            z: 0.,
        });

        (gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z).sqrt()
    }
}

impl TracksForwardTravel for GpsTracking {
    fn forward_travel(&self) -> f64 {
        0.
    }
}
