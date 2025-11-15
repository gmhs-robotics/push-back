use alloc::rc::Rc;
use core::cell::RefCell;

use evian::{
    prelude::*,
    tracking::{RotarySensor, Tracking},
};
use vexide::{
    devices::{math::Point2, smart::GpsSensor},
    prelude::SmartDevice,
    task::{Task, spawn},
    time::{Instant, sleep},
};

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub(crate) struct TrackingData {
    position: Vec2<f64>,
    heading: Angle,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
}

pub struct GpsWheeledTracking<T: RotarySensor + 'static, const N: usize> {
    data: Rc<RefCell<TrackingData>>,
    forward_wheels: [TrackingWheel<T>; N],
    _task: Task<()>,
}

impl<T: RotarySensor + 'static, const N: usize> GpsWheeledTracking<T, N> {
    pub fn new(gps: GpsSensor, forward_wheels: [TrackingWheel<T>; N]) -> Self {
        const {
            assert!(
                N > 0,
                "GPS + Wheeled tracking requires at least one forward tracking wheel to solve forward travel."
            );
        }

        let p_position = gps.position().unwrap_or(Point2 { x: 0., y: 0. });
        let p_heading = gps.heading().map_or(Angle::default(), Angle::from_degrees);

        let data = Rc::new(RefCell::new(TrackingData {
            position: p_position.into(),
            heading: p_heading,
            ..Default::default()
        }));

        let task_data = data.clone();

        let task = spawn(async move {
            let mut p_position = p_position;
            let mut p_time = Instant::now();

            loop {
                sleep(GpsSensor::UPDATE_INTERVAL).await;

                let position = gps.position().unwrap_or(Point2 { x: 0., y: 0. });
                let heading = Angle::from_degrees(gps.heading().unwrap_or_default());

                let time = Instant::now();
                let dt = time.duration_since(p_time).as_secs_f64().max(1e-9);

                let linear_velocity = Vec2::from(position).distance(Vec2::from(p_position)) / dt;
                let angular_velocity = gps
                    .gyro_rate()
                    .map(|rate| rate.z)
                    .unwrap_or_default()
                    .to_radians();

                {
                    let mut d = task_data.borrow_mut();

                    d.position = position.into();
                    d.heading = heading;

                    d.linear_velocity = linear_velocity;
                    d.angular_velocity = angular_velocity;
                }

                p_position = position;
                p_time = time;
            }
        });

        Self {
            data,
            forward_wheels,
            _task: task,
        }
    }
}

impl<T: RotarySensor + 'static, const N: usize> Tracking for GpsWheeledTracking<T, N> {}

impl<T: RotarySensor + 'static, const N: usize> TracksPosition for GpsWheeledTracking<T, N> {
    fn position(&self) -> Vec2<f64> {
        self.data.borrow().position
    }
}

impl<T: RotarySensor + 'static, const N: usize> TracksHeading for GpsWheeledTracking<T, N> {
    fn heading(&self) -> Angle {
        self.data.borrow().heading
    }
}

impl<T: RotarySensor + 'static, const N: usize> TracksVelocity for GpsWheeledTracking<T, N> {
    fn linear_velocity(&self) -> f64 {
        self.data.borrow().linear_velocity
    }

    fn angular_velocity(&self) -> f64 {
        self.data.borrow().angular_velocity
    }
}

impl<T: RotarySensor + 'static, const N: usize> TracksForwardTravel for GpsWheeledTracking<T, N> {
    fn forward_travel(&self) -> f64 {
        let mut sum = 0.0f64;
        let mut cnt = 0usize;

        for w in self.forward_wheels.iter() {
            if let Ok(t) = w.travel() {
                sum += t;
                cnt += 1;
            }
        }

        if cnt == 0 { 0.0 } else { sum / (cnt as f64) }
    }
}

/*
pub struct GpsGyro(GpsSensor);

impl Gyro for GpsGyro {
    type Error = PortError;

    fn heading(&self) -> Result<Angle, Self::Error> {
        Ok(Angle::from_degrees(self.0.heading()?))
    }

    fn angular_velocity(&self) -> Result<f64, Self::Error> {
        Ok(self.0.gyro_rate().map(|rate| rate.z)?.to_radians())
    }
}*/
