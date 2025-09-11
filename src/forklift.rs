use vexide::prelude::Position;

use crate::Robot;

pub trait Forklift {
    async fn lift(
        &mut self,
        level: LiftLevel,
    ) -> Result<(), vexide::devices::smart::motor::MotorError>;
}

pub enum LiftLevel {
    Floor,
    Low,
    Medium,
    High,
}

impl Forklift for Robot {
    async fn lift(
        &mut self,
        level: LiftLevel,
    ) -> Result<(), vexide::devices::smart::motor::MotorError> {
        let position = match level {
            LiftLevel::Floor => Position::from_degrees(0.),
            LiftLevel::Low => Position::from_degrees(360. / 4.),
            LiftLevel::Medium => Position::from_degrees((360. / 4.) * 2.),
            LiftLevel::High => Position::from_degrees((360. / 4.) * 3.),
        };

        self.lift.set_position_target(position, 200)
    }
}
