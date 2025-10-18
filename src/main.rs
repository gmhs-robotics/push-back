#![no_main]
#![no_std]

use core::time::Duration;

use vexide::{
    devices::{math::Point2, smart::GpsSensor},
    prelude::*,
};

use crate::{
    ai::Ai,
    balls::{Intake, Router},
    teams::{ALLIANCE, Alliance, SIDE, Side},
};

extern crate alloc;

mod ai;
mod balls;
mod teams;

pub const MAX_WHEEL: f64 = Motor::V5_MAX_VOLTAGE * 0.85;
pub const MAX_AUTO: f64 = Motor::V5_MAX_VOLTAGE * 0.4;

pub const INCH_TO_METER: f64 = 0.0254;
pub const WHEEL_DIAMETER: f64 = 4. * INCH_TO_METER;
// const TRACK_WIDTH: f64 = 14. * INCH_TO_METER;

// To rotate the body of the robot N degrees, spin the left/right wheels by ROBOT_TO_WHEEL_ROT * N,
// and the opposite side by -ROBOT_TO_WHEEL_ROT * N degrees. Swap which wheels get the negative to
// change turning direction. This works for both radians and degrees, the input and output are
// consistent with units.
// const ROBOT_TO_WHEEL_ROT: f64 = 2100. / 360.0; // TRACK_WIDTH / WHEEL_DIAMETER;

pub struct Robot {
    controller: Controller,

    gps: GpsSensor,

    intake: Intake,
    router: Router,

    left_front: Motor,
    left_back: Motor,
    right_front: Motor,
    right_back: Motor,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        self.intake.forward().ok();
        self.drive_by(1.2).await;
        self.intake.disable().ok();

        match ALLIANCE {
            Alliance::Red => self.rotate_to(270.).await,
            Alliance::Blue => self.rotate_to(90.).await,
        }

        self.drive_by(0.6).await;

        match (ALLIANCE, SIDE) {
            (Alliance::Blue, Side::Left) => self.rotate_to(180.).await,
            (Alliance::Blue, Side::Right) => self.rotate_to(0.).await,
            (Alliance::Red, Side::Left) => self.rotate_to(0.).await,
            (Alliance::Red, Side::Right) => self.rotate_to(180.).await,
        }

        self.drive_by(0.6).await;

        match ALLIANCE {
            Alliance::Red => self.rotate_to(270.).await,
            Alliance::Blue => self.rotate_to(90.).await,
        }

        self.drive_by(-0.5).await;

        self.intake.reverse().ok();
        self.router.reverse().ok();

        sleep(Duration::from_secs(3)).await;

        self.intake.disable().ok();
        self.router.disable().ok();
    }

    async fn driver(&mut self) {
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
                self.intake.forward().ok();
            } else if int_bw && !int_fw {
                self.intake.reverse().ok();
            } else {
                self.intake.disable().ok();
            }

            if rou_fw && !rou_bw {
                self.router.forward().ok();
            } else if rou_bw && !rou_fw {
                self.router.reverse().ok();
            } else {
                self.router.disable().ok();
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let controller = peripherals.primary_controller;

    let left_front = Motor::new(peripherals.port_10, Gearset::Green, Direction::Forward);
    let left_back = Motor::new(peripherals.port_9, Gearset::Green, Direction::Forward);
    let right_front = Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse);
    let right_back = Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse);

    // Must move inverse to eachother
    let intake = Motor::new(peripherals.port_11, Gearset::Green, Direction::Forward);
    let outtake = Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse);

    // Decides which hole the balls go out
    let router = Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward);

    // let mut display = peripherals.display;

    // display.set_render_mode(RenderMode::DoubleBuffered);

    let gps = GpsSensor::new(
        peripherals.port_20,
        Point2 { x: 0., y: -0.08 },
        Point2 { x: 1.41, y: -0.7 },
        270.,
    );

    let robot = Robot {
        controller,

        intake: Intake { intake, outtake },
        router: Router { router },

        gps,

        // display,
        left_front,
        left_back,
        right_front,
        right_back,
    };

    robot.compete().await;
}
