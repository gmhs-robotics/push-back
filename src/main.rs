#![feature(trait_alias)]

use bincode::{Decode, Encode};
use vexide::{adi::digital::LogicLevel, prelude::*, smart::PortError};

use crate::{
    mechanisms::{Ternary, TernaryMotor},
    record::frame::Recordable,
    sdcard::is_sdcard_inserted,
};

mod mechanisms;
mod record;
mod sdcard;

pub const MAX_WHEEL: f64 = Motor::V5_MAX_VOLTAGE;

struct Robot {
    primary_controller: Controller,

    left: [Motor; 3],
    right: [Motor; 3],

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

impl Recordable for Robot {
    type Frame = RobotFrame;
    const UPDATE_INTERVAL: std::time::Duration = Controller::UPDATE_INTERVAL;

    async fn get_new_frame(&self) -> Self::Frame {
        let controller_state = self.primary_controller.state().unwrap_or_default();

        let t = controller_state.right_stick.y();
        let r = controller_state.left_stick.x();

        let left_volts = (t + r) * MAX_WHEEL;
        let right_volts = (t - r) * MAX_WHEEL;

        let r1 = controller_state.button_r1.is_pressed();
        let r2 = controller_state.button_r2.is_pressed();
        let l1_now = controller_state.button_l1.is_now_pressed();

        // r1 r2
        // T  F => -O -I
        // F  T => +O +I
        // T  T => -O +I
        // F  F =>  I  O
        //
        // l2   => +P
        // l1   => -P

        let (intake, outake) = match (r2, r1) {
            (true, false) => (Ternary::Low, Ternary::Low),
            (false, true) => (Ternary::High, Ternary::High),
            (true, true) => (Ternary::High, Ternary::Low),
            (false, false) => (Ternary::Zero, Ternary::Zero),
        };

        let piston_state = self.piston.is_high().unwrap_or_default();

        let target_piston_state = if l1_now { !piston_state } else { piston_state };
        let outake = if target_piston_state { outake } else { !outake };

        let (intake_volts, outake_volts) = (
            self.intake.calculate_from_ternary(intake),
            self.outake.calculate_from_ternary(outake),
        );

        Self::Frame {
            left: left_volts,
            right: right_volts,
            intake: intake_volts,
            outake: outake_volts,
            piston_state: target_piston_state,
        }
    }

    async fn transform_to_frame(&mut self, frame: &Self::Frame) -> Result<(), PortError> {
        for motor in &mut self.left {
            let _ = motor.set_voltage(frame.left);
        }

        for motor in &mut self.right {
            let _ = motor.set_voltage(frame.right);
        }

        let _ = self.intake.motor.set_voltage(frame.intake);
        let _ = self.outake.motor.set_voltage(frame.outake);
        let _ = self.piston.set_level(frame.piston_state.into());

        Ok(())
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    if !is_sdcard_inserted() {
        panic!("SD Card not inserted");
    }

    let primary_controller = peripherals.primary_controller;
    // let partner_controller = peripherals.partner_controller;

    let left = [
        Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
    ];
    let right = [
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
    ];

    let intake = TernaryMotor::new(
        Motor::new(peripherals.port_3, Gearset::Green, Direction::Forward),
        MAX_WHEEL,
    );
    let outake = TernaryMotor::new(
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
        MAX_WHEEL,
    );

    let display = peripherals.display;

    let piston = AdiDigitalOut::with_initial_level(peripherals.adi_h, LogicLevel::High);

    let robot = Robot {
        primary_controller,
        // partner_controller,
        left,
        right,
        intake,
        outake,

        piston,
    };

    if cfg!(feature = "record") {
        record::runtime::RecordingAutonomous::compete(robot, display).await;
    } else {
        record::runtime::PlaybackAutonomous::compete(robot, display).await;
    };
}
