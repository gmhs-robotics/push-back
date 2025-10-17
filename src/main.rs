#![no_main]
#![no_std]

use core::time::Duration;

use crate::ai::Ai;
use vexide::{
    devices::{
        display::{self, Font, FontSize, RenderMode, Text},
        math::Point2,
        smart::GpsSensor,
    },
    prelude::*,
};

extern crate alloc;

mod ai;

pub const MAX_WHEEL: f64 = Motor::V5_MAX_VOLTAGE * 0.85;
pub const MAX_AUTO: f64 = Motor::V5_MAX_VOLTAGE * 0.4;

// All in metres
const INCH_TO_METER: f64 = 0.0254;
const WHEEL_DIAMETER: f64 = 4. * INCH_TO_METER;
// const TRACK_WIDTH: f64 = 14. * INCH_TO_METER;

// To rotate the body of the robot N degrees, spin the left/right wheels by ROBOT_TO_WHEEL_ROT * N,
// and the opposite side by -ROBOT_TO_WHEEL_ROT * N degrees. Swap which wheels get the negative to
// change turning direction. This works for both radians and degrees, the input and output are
// consistent with units.
const ROBOT_TO_WHEEL_ROT: f64 = 2100. / 360.0; // TRACK_WIDTH / WHEEL_DIAMETER;

pub struct Robot {
    controller: Controller,

    // https://pros.cs.purdue.edu/v5/_images/gps_get_position.jpg
    gps: GpsSensor,

    display: Display,

    intake: Motor,
    outtake: Motor,
    router: Motor,

    left_front: Motor,
    left_back: Motor,
    right_front: Motor,
    right_back: Motor,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("[competition.autonomous] rec autonomous mode");

        self.rotate_to(180.).await;
        sleep(Duration::from_secs(2)).await;
        self.rotate_to(0.).await;

        /* self.left_front.set_voltage(0.).ok();
        self.left_back.set_voltage(0.).ok();
        self.right_front.set_voltage(0.).ok();
        self.right_back.set_voltage(0.).ok();

        let mut buffer = ryu::Buffer::new();

        self.left_front.set_voltage(MAX_AUTO).ok();
        self.left_back.set_voltage(MAX_AUTO).ok();
        self.right_front.set_voltage(-MAX_AUTO).ok();
        self.right_back.set_voltage(-MAX_AUTO).ok();

        loop {
            let rotation = match self.gps.heading() {
                Ok(r) => r,
                Err(_) => {
                    continue;
                }
            };

            if rotation > 165. && rotation < 185. {
                break;
            }

            self.display.erase(Rgb::new(0, 0, 0));

            self.display.draw_text(
                &Text::new(
                    buffer.format(rotation.ceil()),
                    Font::new(FontSize::EXTRA_LARGE, display::FontFamily::Monospace),
                    Point2 { x: 0, y: 0 },
                ),
                Rgb::new(255, 255, 255),
                None,
            );

            self.display.render();

            sleep(GpsSensor::UPDATE_INTERVAL + Display::REFRESH_INTERVAL).await;
        }

        self.left_front.set_voltage(0.).ok();
        self.left_back.set_voltage(0.).ok();
        self.right_front.set_voltage(0.).ok();
        self.right_back.set_voltage(0.).ok();

        loop {
            let rotation = match self.gps.heading() {
                Ok(r) => r,
                Err(_) => {
                    continue;
                }
            };

            self.display.erase(Rgb::new(0, 0, 0));

            self.display.draw_text(
                &Text::new(
                    buffer.format(rotation.ceil()),
                    Font::new(FontSize::EXTRA_LARGE, display::FontFamily::Monospace),
                    Point2 { x: 0, y: 0 },
                ),
                Rgb::new(255, 255, 255),
                None,
            );

            self.display.render();

            sleep(GpsSensor::UPDATE_INTERVAL + Display::REFRESH_INTERVAL).await;
        }*/

        /*loop {
            let rotation = match self.gps.position() {
                Ok(r) => r.x,
                Err(_) => -6.0,
            };

            let mut buffer = ryu::Buffer::new();
            self.display.draw_text(
                &Text::new(
                    buffer.format(rotation),
                    Font::new(FontSize::EXTRA_LARGE, display::FontFamily::Monospace),
                    Point2 { x: 100, y: 100 },
                ),
                Rgb::new(255, 255, 255),
                None,
            );

            sleep(GpsSensor::UPDATE_INTERVAL).await;
        }*/

        /* let path: &GoalPath = &[
            Goal {
                position: Point2 { x: 0.5, y: 0.5 },
                rotation: 180.,
            },
            /*            Goal {
                position: Point2 { x: 0., y: 0. },
                rotation: 0.,
            },*/
        ];

        for goal in path {
            loop {
                let position = match self.gps.position() {
                    Ok(p) => p,
                    Err(_) => {
                        continue;
                    }
                };

                let rotation = match self.gps.heading() {
                    Ok(h) => h,
                    Err(_) => {
                        continue;
                    }
                };

                const RPM: i32 = 100;

                {
                    // Rotation logic
                    let dr = goal.rotation - rotation;
                    println!("to rotate (Z-AXIS): {dr}");

                    // Calculate the shortest direction of rotation (clockwise or counter-clockwise)
                    let dr = if dr > 180.0 {
                        dr - 360.0 // Rotate counter-clockwise
                    } else if dr < -180.0 {
                        dr + 360.0 // Rotate clockwise
                    } else {
                        dr // No adjustment needed
                    };

                    println!("Optimized rotation angle: {dr}");

                    let wheel_rot = ROBOT_TO_WHEEL_ROT * dr;
                    println!("to rotate (wheel): {wheel_rot}");

                    let time = Duration::from_secs_f64(wheel_rot / (RPM as f64 * 6.0));

                    // Reset positions
                    self.left_front.reset_position().ok();
                    self.left_back.reset_position().ok();
                    self.right_front.reset_position().ok();
                    self.right_back.reset_position().ok();

                    let left_target =
                        MotorControl::Position(Position::from_degrees(-wheel_rot), RPM);
                    let right_target =
                        MotorControl::Position(Position::from_degrees(wheel_rot), RPM);

                    self.left_front.set_target(left_target).ok();
                    self.left_back.set_target(left_target).ok();
                    self.right_front.set_target(right_target).ok();
                    self.right_back.set_target(right_target).ok();

                    // Wait for the rotation to complete
                    sleep(time).await;
                }

                // Drive logic
                {
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
                }

                break;
            }
        }*/
    }

    async fn driver(&mut self) {
        println!("[competition.driver] rec driver mode");

        loop {
            let controller_state = self.controller.state().unwrap_or_default();

            // Forward/Back
            let lx = controller_state.right_stick.x();

            // Strafe
            let ly = controller_state.right_stick.y();

            // Rotation
            let rx = controller_state.left_stick.x();

            let v_fl = ly - lx + rx;
            let v_fr = ly + lx - rx;
            let v_bl = ly + lx + rx;
            let v_br = ly - lx - rx;

            self.left_front.set_voltage(v_fl * MAX_WHEEL).ok();
            self.left_back.set_voltage(v_bl * MAX_WHEEL).ok();
            self.right_front.set_voltage(v_fr * MAX_WHEEL).ok();
            self.right_back.set_voltage(v_br * MAX_WHEEL).ok();

            let int_fw = controller_state.button_r1.is_pressed();
            let int_bw = controller_state.button_l1.is_pressed();

            let rou_fw = controller_state.button_r2.is_pressed();
            let rou_bw = controller_state.button_l2.is_pressed();

            if int_fw && !int_bw {
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE).ok();
                self.outtake.set_voltage(Motor::V5_MAX_VOLTAGE).ok();
            } else if int_bw && !int_fw {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE).ok();
                self.outtake.set_voltage(-Motor::V5_MAX_VOLTAGE).ok();
            } else {
                self.intake.set_voltage(0.).ok();
                self.outtake.set_voltage(0.).ok();
            }

            if rou_fw && !rou_bw {
                self.router.set_voltage(Motor::V5_MAX_VOLTAGE).ok();
            } else if rou_bw && !rou_fw {
                self.router.set_voltage(-Motor::V5_MAX_VOLTAGE).ok();
            } else {
                self.router.set_voltage(0.).ok();
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    println!("[init.begin] starting init");

    let controller = peripherals.primary_controller;

    println!("[init.motor] init LF(10), LB(9), RF(1), RB(2), IN(11), OUT(3), ROU(4) on ports");
    let left_front = Motor::new(peripherals.port_10, Gearset::Green, Direction::Forward);
    let left_back = Motor::new(peripherals.port_9, Gearset::Green, Direction::Forward);
    let right_front = Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse);
    let right_back = Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse);

    // Must move inverse to eachother
    let intake = Motor::new(peripherals.port_11, Gearset::Green, Direction::Forward);
    let outtake = Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse);

    // Decides which hole the balls go out
    let router = Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward);

    let mut display = peripherals.display;

    display.set_render_mode(RenderMode::DoubleBuffered);

    let gps = GpsSensor::new(
        peripherals.port_20,
        Point2 { x: 0., y: -0.08 },
        Point2 { x: 1.41, y: -0.7 },
        270.,
    );

    let robot = Robot {
        controller,

        intake,
        outtake,
        router,

        gps,

        display,

        left_front,
        left_back,
        right_front,
        right_back,
    };

    println!("[init.ready] Ready, entering competition mode");
    robot.compete().await;
}
