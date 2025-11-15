use alloc::rc::Rc;
use core::cell::RefCell;

use evian::{prelude::*, tracking::RotarySensor};
use vexide::{
    devices::{
        math::{Point2, Vector3},
        smart::GpsSensor,
    },
    float::Float,
    prelude::{Motor, SmartDevice},
    task::{Task, spawn},
    time::{Instant, sleep},
};

const VEC3_ZEROS: Vector3<f64> = Vector3 {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub(crate) struct TrackingData {
    position: Vec2<f64>,
    heading: Angle,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
}

pub struct GpsWheeledTracking {
    data: Rc<RefCell<TrackingData>>,
    _task: Task<()>,
}

impl GpsWheeledTracking {
    pub fn new<T: RotarySensor + 'static, const N_FORWARD: usize>(
        gps: GpsSensor,
        forward_wheels: [TrackingWheel<T>; N_FORWARD],
    ) -> Self {
        let p_position = gps.position().unwrap_or(Point2 { x: 0., y: 0. });
        let p_heading = gps.heading().map_or(Angle::default(), Angle::from_degrees);

        let data = Rc::new(RefCell::new(TrackingData {
            position: p_position.into(),
            heading: p_heading,
            ..Default::default()
        }));

        let c_data = data.clone();

        let task = spawn(async move {
            let mut p_position = p_position;
            let mut p_time = Instant::now();

            loop {
                sleep(Motor::WRITE_INTERVAL.max(GpsSensor::UPDATE_INTERVAL)).await;

                let time = Instant::now();
                let dt = time.duration_since(p_time).as_secs_f64().max(1e-9);

                let travel: [f64; N_FORWARD] =
                    core::array::from_fn(|i| forward_wheels[i].travel().unwrap_or_default());

                // compute average total travel across available wheels (WheeledTracking does this)
                let mut travel_sum = 0.0;
                let mut travel_count = 0usize;

                for t in travel {
                    travel_sum += t;
                    travel_count += 1;
                }

                let avg_total_travel = if travel_count != 0 {
                    travel_sum / (travel_count as f64)
                } else {
                    0.0
                };

                let position = gps.position().unwrap_or(Point2 { x: 0., y: 0. });
                let heading = Angle::from_degrees(gps.heading().unwrap_or_default());

                // compute gps-based linear velocity if possible
                let dx = position.x - p_position.x;
                let dy = position.y - p_position.y;

                let gps_linear_vel = (dx * dx + dy * dy).sqrt() / dt;

                // write authoritative GPS pose/heading/vel, and wheel-derived forward_travel
                {
                    let mut d = c_data.borrow_mut();
                    d.forward_travel = avg_total_travel;
                    d.position = position.into();
                    d.heading = heading;
                    d.linear_velocity = gps_linear_vel;
                    d.angular_velocity = gps.gyro_rate().map(|rate| rate.z).unwrap_or_default();
                }

                // update prev snapshots & time
                p_position = position;
                p_time = time;
            }
        });

        Self { data, _task: task }
    }
}

use evian::tracking::Tracking;

impl Tracking for GpsWheeledTracking {}

impl TracksPosition for GpsWheeledTracking {
    fn position(&self) -> Vec2<f64> {
        self.data.borrow().position
    }
}

impl TracksHeading for GpsWheeledTracking {
    fn heading(&self) -> Angle {
        self.data.borrow().heading
    }
}

impl TracksVelocity for GpsWheeledTracking {
    fn linear_velocity(&self) -> f64 {
        self.data.borrow().linear_velocity
    }

    fn angular_velocity(&self) -> f64 {
        self.data.borrow().angular_velocity
    }
}

impl TracksForwardTravel for GpsWheeledTracking {
    fn forward_travel(&self) -> f64 {
        self.data.borrow().forward_travel
    }
}
