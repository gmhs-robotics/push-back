#![no_main]
#![no_std]

mod forklift;
mod odometry;

use nalgebra::Vector3;
use vexide::prelude::*;

use crate::{
    forklift::{Forklift, LiftLevel},
    odometry::MecanumOdometry,
};

pub struct Robot {
    controller: Controller,

    lift: Motor,

    left_front: Motor,
    left_back: Motor,
    right_front: Motor,
    right_back: Motor,

    odometry: MecanumOdometry,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("[competition.autonomous] rec autonomous mode");

        // TODO: Implement our game plan
    }

    async fn driver(&mut self) {
        println!("[competition.driver] rec driver mode");

        loop {
            let controller_state = self.controller.state().unwrap_or_default();

            // Forward/Back
            let lx = controller_state.left_stick.y();

            // Strafe
            let ly = controller_state.left_stick.y();

            // Rotation
            let rx = controller_state.right_stick.x();

            let v_fl = ly - lx - rx;
            let v_fr = ly + lx + rx;
            let v_bl = ly + lx - rx;
            let v_br = ly - lx + rx;

            self.left_front
                .set_voltage(v_fl * Motor::V5_MAX_VOLTAGE)
                .ok();
            self.left_back
                .set_voltage(v_bl * Motor::V5_MAX_VOLTAGE)
                .ok();
            self.right_front
                .set_voltage(v_fr * Motor::V5_MAX_VOLTAGE)
                .ok();
            self.right_back
                .set_voltage(v_br * Motor::V5_MAX_VOLTAGE)
                .ok();

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

            let lf = self
                .left_front
                .raw_position()
                .map(|x| x.0)
                .unwrap_or_default();
            let rf = self
                .right_front
                .raw_position()
                .map(|x| x.0)
                .unwrap_or_default();
            let lb = self
                .left_back
                .raw_position()
                .map(|x| x.0)
                .unwrap_or_default();
            let rb = self
                .right_back
                .raw_position()
                .map(|x| x.0)
                .unwrap_or_default();

            let pose = self.odometry.update(lf, rf, lb, rb);

            println!("{}", pose);

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    println!("[init.begin] starting init");

    let controller = peripherals.primary_controller;

    println!("[init.motor] init LF(12), LB(11), RF(2), RB(1), LIFT(5) on ports");
    let left_front = Motor::new(peripherals.port_12, Gearset::Green, Direction::Forward);
    let left_back = Motor::new(peripherals.port_11, Gearset::Green, Direction::Forward);
    let right_front = Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse);
    let right_back = Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse);

    let lift = Motor::new(peripherals.port_5, Gearset::Green, Direction::Forward);

    let robot = Robot {
        controller,

        odometry: MecanumOdometry::new(
            [
                left_front.raw_position().map(|x| x.0).unwrap_or_default(),
                right_front.raw_position().map(|x| x.0).unwrap_or_default(),
                left_back.raw_position().map(|x| x.0).unwrap_or_default(),
                right_back.raw_position().map(|x| x.0).unwrap_or_default(),
            ],
            Vector3::zeros(),
        ),

        lift,
        left_front,
        left_back,
        right_front,
        right_back,
    };

    println!("[init.ready] Ready, entering competition mode");
    robot.compete().await;
}
