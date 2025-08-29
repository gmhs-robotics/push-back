#![no_main]
#![no_std]

use vexide::prelude::*;

struct Robot {
    controller: Controller,

    left_front: Motor,
    left_back: Motor,
    right_front: Motor,
    right_back: Motor,
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

    let robot = Robot {
        controller,

        left_front,
        left_back,
        right_front,
        right_back,
    };

    println!("[init.ready] Ready, entering competition mode");
    robot.compete().await;
}
