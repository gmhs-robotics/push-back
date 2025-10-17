use crate::{MAX_AUTO, Robot};
use vexide::{
    devices::{math::Point2, smart::GpsSensor},
    prelude::*,
};

pub trait Ai {
    async fn rotate_to(&mut self, deg: f64);
    async fn drive_to(&mut self, point: Point2<f64>);
}

impl Ai for Robot {
    async fn rotate_to(&mut self, deg: f64) {
        self.left_front.set_voltage(MAX_AUTO).ok();
        self.left_back.set_voltage(MAX_AUTO).ok();
        self.right_front.set_voltage(-MAX_AUTO).ok();
        self.right_back.set_voltage(-MAX_AUTO).ok();

        loop {
            let rotation = match self.gps.heading() {
                Ok(r) => r % 360.0,
                Err(_) => continue,
            };

            let diff = ((deg - rotation + 540.0) % 360.0) - 180.0;

            if diff > -15.0 && diff < 15.0 {
                break;
            }

            sleep(GpsSensor::UPDATE_INTERVAL).await;
        }

        self.left_front.set_voltage(0.).ok();
        self.left_back.set_voltage(0.).ok();
        self.right_front.set_voltage(0.).ok();
        self.right_back.set_voltage(0.).ok();
    }

    async fn drive_to(&mut self, goal: Point2<f64>) {
        /* loop {
        let position = match self.gps.position() {
                        Ok(r) => r,
                        Err(_) => continue,
                    };

                            let dx = goal.position.x - position.x;
                            let dy = goal.position.y - position.y;
                            let dxy = (dx * dx + dy * dy).sqrt();

                            let revolutions = dxy / (PI * WHEEL_DIAMETER);

                            let drive_target =
                                MotorControl::Position(Position::from_revolutions(revolutions), RPM);

                            // Reset positions
                            self.left_front.reset_position().ok();
                            self.left_back.reset_position().ok();
                            self.right_front.reset_position().ok();
                            self.right_back.reset_position().ok();

                            self.left_front.set_target(drive_target).ok();
                            self.left_back.set_target(drive_target).ok();
                            self.right_front.set_target(drive_target).ok();
                            self.right_back.set_target(drive_target).ok();

                            // Calculate time to drive, ensuring it's non-negative
                            let time = Duration::from_secs_f64((revolutions / RPM as f64) * 60.0);

                            // Wait for the drive to complete
                            sleep(time).await;
                }*/
    }
}
