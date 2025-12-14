#![feature(trait_alias)]

use autons::prelude::*;
use bincode::{Decode, Encode};
use vexide::{adi::digital::LogicLevel, prelude::*};

use crate::{
    mechanisms::TernaryMotor,
    record::{
        frame::Recordable,
        runtime::AutonomousRecorder,
    },
    sdcard::is_sdcard_inserted,
};

mod mechanisms;
mod record;
mod sdcard;

pub const MAX_WHEEL: f64 = Motor::V5_MAX_VOLTAGE;

struct Robot {
    controller: Controller,

    left: [Motor; 2],
    right: [Motor; 2],

    intake: TernaryMotor,
    outake: TernaryMotor,

    piston: AdiDigitalOut,
}

#[derive(Encode, Decode, Default, Clone)]
struct RobotFrame {
    left: f64,
    right: f64,

    intake: f64,
    outake: f64,

    piston_state: bool,
}

impl SelectCompete for Robot {
    async fn driver(&mut self) {
        loop {
            let frame = self.get_new_frame().await;
            self.transform_to_frame(&frame).await;

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }

    async fn disabled(&mut self) {}
}

impl Recordable for Robot {
    type Frame = RobotFrame;

    async fn get_new_frame(&self) -> Self::Frame {
        let controller_state = self.controller.state().unwrap_or_default();

        let t = controller_state.right_stick.y();
        let r = controller_state.left_stick.x();

        let left_volts = (t + r) * MAX_WHEEL;
        let right_volts = (t - r) * MAX_WHEEL;

        let intake_volts = self.intake.calculate_from_button_state(
            controller_state.button_r2.is_pressed(),
            controller_state.button_l2.is_pressed(),
        );

        let outake_volts = self.outake.calculate_from_button_state(
            controller_state.button_r1.is_pressed(),
            controller_state.button_x.is_pressed(),
        );

        let toggle_piston = controller_state.button_l1.is_now_pressed();

        let current_piston_state = self.piston.is_high().unwrap_or_default();

        let target_piston_state = if toggle_piston {
            !current_piston_state
        } else {
            current_piston_state
        };

        Self::Frame {
            left: left_volts,
            right: right_volts,
            intake: intake_volts,
            outake: outake_volts,
            piston_state: target_piston_state,
        }
    }

    async fn transform_to_frame(&mut self, frame: &Self::Frame) {
        for motor in &mut self.left {
            let _ = motor.set_voltage(frame.left);
        }

        for motor in &mut self.right {
            let _ = motor.set_voltage(frame.right);
        }

        let _ = self.intake.motor.set_voltage(frame.intake);
        let _ = self.outake.motor.set_voltage(frame.outake);

        let _ = self.piston.set_level(if frame.piston_state {
            LogicLevel::High
        } else {
            LogicLevel::Low
        });
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    if !is_sdcard_inserted() {
        panic!("SD Card not inserted");
    }

    let controller = peripherals.primary_controller;
    let left = [
        Motor::new(peripherals.port_18, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_12, Gearset::Green, Direction::Forward),
    ];
    let right = [
        Motor::new(peripherals.port_10, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_9, Gearset::Green, Direction::Reverse),
    ];

    let intake = TernaryMotor::new(
        Motor::new(peripherals.port_19, Gearset::Green, Direction::Forward),
        MAX_WHEEL,
    );
    let outake = TernaryMotor::new(
        Motor::new(peripherals.port_21, Gearset::Green, Direction::Forward),
        MAX_WHEEL,
    );

    let display = peripherals.display;

    let piston = AdiDigitalOut::with_initial_level(peripherals.adi_a, LogicLevel::High);

    let robot = Robot {
        controller,

        left,
        right,
        intake,
        outake,

        piston,
    };

    #[cfg(feature = "record")]
    let (recorder, selector) = AutonomousRecorder::new(
        robot,
        display,
        Controller::UPDATE_INTERVAL,
        crate::record::selector::RecordTarget::New,
    );

    #[cfg(not(feature = "record"))]
    let (recorder, selector) =
        AutonomousRecorder::new(robot, display, Controller::UPDATE_INTERVAL);

    recorder.compete(selector).await;
}
