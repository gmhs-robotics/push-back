#![no_main]
#![no_std]

use vexide::{
    devices::{math::Point2, smart::GpsSensor},
    prelude::*,
};

const MAX_WHEEL: f64 = Motor::V5_MAX_VOLTAGE * 0.8;

pub struct Robot {
    controller: Controller,

    gps: GpsSensor,

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

        // TODO: Implement our game plan
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

    let gps = GpsSensor::new(
        peripherals.port_15,
        Point2 { x: 0., y: 0. },
        Point2 { x: 0., y: 0. },
        180.,
    );

    let robot = Robot {
        controller,

        intake,
        outtake,
        router,

        gps,

        left_front,
        left_back,
        right_front,
        right_back,
    };

    println!("[init.ready] Ready, entering competition mode");
    robot.compete().await;
}
