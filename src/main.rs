#![no_main]
#![no_std]

extern crate alloc;

use autons::{
    prelude::*,
    simple::{route, SimpleSelect},
};
use vexide::prelude::*;

use crate::mechanisms::BinaryMotor;

mod mechanisms;

pub const MAX_WHEEL: f64 = Motor::V5_MAX_VOLTAGE;
pub const MAX_MECH: f64 = Motor::V5_MAX_VOLTAGE;

struct Robot {
    controller: Controller,

    left: Motor,
    right: Motor,

    intake: BinaryMotor,
    outtake: BinaryMotor,

    piston: AdiDigitalOut,
}

impl Robot {
    async fn route_red_left(&mut self) {}
    async fn route_red_right(&mut self) {}
    async fn route_blue_left(&mut self) {}
    async fn route_blue_right(&mut self) {}
}

impl SelectCompete for Robot {
    async fn driver(&mut self) {
        loop {
            let controller_state = self.controller.state().unwrap_or_default();

            let t = controller_state.right_stick.y();
            let r = controller_state.left_stick.x();

            self.left.set_voltage((t - r) * MAX_WHEEL).ok();
            self.right.set_voltage((t + r) * MAX_WHEEL).ok();

            let intake_forward = controller_state.button_r2.is_pressed();
            let intake_reverse = controller_state.button_l2.is_pressed();

            self.intake
                .update_from_button_state(intake_forward, intake_reverse)
                .ok();

            let outake_forward = controller_state.button_r1.is_pressed();

            if outake_forward {
                self.outtake.forward().ok();
            } else {
                self.outtake.disable().ok();
            }

            let toggle_piston = controller_state.button_l1.is_now_pressed();

            if toggle_piston {
                if self.piston.is_high().unwrap_or_default() {
                    self.piston.set_low().ok();
                } else {
                    self.piston.set_high().ok();
                }
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let controller = peripherals.primary_controller;

    let left = Motor::new(peripherals.port_18, Gearset::Green, Direction::Forward);
    let right = Motor::new(peripherals.port_10, Gearset::Green, Direction::Reverse);

    let intake = BinaryMotor(
        Motor::new(peripherals.port_19, Gearset::Green, Direction::Forward),
        MAX_MECH,
    );
    let outtake = BinaryMotor(
        Motor::new(peripherals.port_21, Gearset::Green, Direction::Forward),
        MAX_MECH,
    );

    let piston = AdiDigitalOut::new(peripherals.adi_a);

    let robot = Robot {
        controller,

        left,
        right,
        intake,
        outtake,

        piston,
    };

    robot
        .compete(SimpleSelect::new(
            peripherals.display,
            [
                route!("Red, Left", Robot::route_red_left),
                route!("Red, Right", Robot::route_red_right),
                route!("Blue, Left", Robot::route_blue_left),
                route!("Blue, Right", Robot::route_blue_right),
            ],
        ))
        .await;
}
