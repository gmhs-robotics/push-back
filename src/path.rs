use vexide::devices::math::Point2;

pub type GoalPath = [Goal];

pub struct Goal {
    pub position: Point2<f64>,
    pub rotation: f64,
}
