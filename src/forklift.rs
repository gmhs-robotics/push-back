pub trait Forklift {
    async fn lift(&mut self, level: LiftLevel);
}

pub enum LiftLevel {
    Floor,
    Low,
    Medium,
    High,
}
