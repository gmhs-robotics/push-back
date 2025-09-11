#![no_main]
#![no_std]

mod forklift;

use vexide::prelude::*;

use crate::forklift::{Forklift, LiftLevel};

struct Robot {
    controller: Controller,

    lift: Motor,

    left_front: Motor,
    left_back: Motor,
    right_front: Motor,
    right_back: Motor,
}

impl Forklift for Robot {
    async fn lift(
        &mut self,
        level: forklift::LiftLevel,
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

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("[competition.autonomous] rec autonomous mode");

        // TODO: Implement our game plan
    }

    async fn driver(&mut self) {
        println!("[competition.driver] rec driver mode");

        // simple mecanum drive
        loop {
            let controller_state = self.controller.state().unwrap_or_default();

            // LY
            let axis3 = controller_state.left_stick.y();
            // LX
            let axis4 = controller_state.left_stick.x();
            // RX
            let axis1 = controller_state.right_stick.x();

            let lf = axis3 + axis4 + axis1;
            let rf = axis3 - axis4 - axis1;
            let lb = axis3 - axis4 + axis1;
            let rb = axis3 + axis4 - axis1;

            self.left_front.set_voltage(lf * Motor::V5_MAX_VOLTAGE).ok();
            self.right_front
                .set_voltage(rf * Motor::V5_MAX_VOLTAGE)
                .ok();
            self.left_back.set_voltage(lb * Motor::V5_MAX_VOLTAGE).ok();
            self.right_back.set_voltage(rb * Motor::V5_MAX_VOLTAGE).ok();

            let level = if controller_state.button_a.is_now_pressed() {
                Some(LiftLevel::High)
            } else if controller_state.button_b.is_now_pressed() {
                Some(LiftLevel::Medium)
            } else if controller_state.button_x.is_now_pressed() {
                Some(LiftLevel::Low)
            } else if controller_state.button_y.is_now_pressed() {
                Some(LiftLevel::Floor)
            } else {
                None
            };

            if let Some(lift_level) = level {
                self.lift(lift_level).await.ok();
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    println!("[init.begin] starting init");

    let controller = peripherals.primary_controller;

    println!("[init.motor] init LF(1), LB(2), RF(3), RB(4) on ports");
    let left_front = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    let left_back = Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward);
    let right_front = Motor::new(peripherals.port_3, Gearset::Green, Direction::Forward);
    let right_back = Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward);

    let lift = Motor::new(peripherals.port_5, Gearset::Green, Direction::Forward);

    let robot = Robot {
        controller,

        lift,

        left_front,
        left_back,
        right_front,
        right_back,
    };

    println!("[init.ready] Ready, entering competition mode");
    robot.compete().await;
}
