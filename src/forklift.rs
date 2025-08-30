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
