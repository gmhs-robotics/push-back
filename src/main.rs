#![feature(trait_alias)]

use std::time::Duration;

use ozton::{
    derive::RecordedRobot,
    drivetrain::model::Differential,
    prelude::{Drivetrain, NoTracking, RecordableDrivetrain},
    record::{
        DifferentialVoltageFrame, Recordable,
        runtime::{PlaybackAutonomous, RecordingAutonomous},
    },
};
use vexide::{adi::digital::LogicLevel, prelude::*};

use crate::{
    mechanisms::{Ternary, TernaryMotor, adi_toggle_pure},
    sdcard::is_sdcard_inserted,
};

mod mechanisms;
mod sdcard;

pub const MAX_WHEEL: f64 = Motor::V5_MAX_VOLTAGE;
const RECORD_DRIVETRAIN_LIMIT: f64 = 0.3;

#[derive(RecordedRobot)]
struct Robot {
    #[record(skip)]
    primary_controller: Controller,

    drivetrain: RecordableDrivetrain<Differential, NoTracking>,
    intake: TernaryMotor,
    outake: TernaryMotor,

    switch: AdiDigitalOut,
    descore: AdiDigitalOut,
}

#[ozton::record::async_trait(?Send)]
impl Recordable for Robot {
    const UPDATE_INTERVAL: Duration = Controller::UPDATE_INTERVAL;

    async fn get_new_frame(&self) -> Self::Frame {
        let controller_state = self.primary_controller.state().unwrap_or_default();

        let throttle = -controller_state.right_stick.y();
        let turn = controller_state.left_stick.x();

        let r1 = controller_state.button_r1.is_pressed();
        let r2 = controller_state.button_r2.is_pressed();

        let l1_now = controller_state.button_l1.is_now_pressed();
        let l2_now = controller_state.button_l2.is_now_pressed();

        let (intake, outake) = match (r2, r1) {
            (true, false) => (Ternary::Low, Ternary::Low),
            (false, true) => (Ternary::High, Ternary::High),
            (true, true) => (Ternary::High, Ternary::Low),
            (false, false) => (Ternary::Zero, Ternary::Zero),
        };

        let switch = adi_toggle_pure(&self.switch, l1_now).unwrap_or_default();
        let descore = adi_toggle_pure(&self.descore, l2_now).unwrap_or_default();

        let outake = if switch { outake } else { !outake };

        let mut drivetrain = DifferentialVoltageFrame::arcade(throttle, turn);
        if cfg!(feature = "record") {
            drivetrain.left *= RECORD_DRIVETRAIN_LIMIT;
            drivetrain.right *= RECORD_DRIVETRAIN_LIMIT;
        }

        Self::Frame {
            drivetrain,
            intake: self.intake.calculate_from_ternary(intake),
            outake: self.outake.calculate_from_ternary(outake),
            switch,
            descore,
        }
    }

    async fn on_save(&mut self) {
        let _ = self.primary_controller.rumble(".").await;
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    if !is_sdcard_inserted() {
        panic!("SD Card not inserted");
    }

    let left = [
        Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
    ];
    let right = [
        Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward),
    ];

    let intake = TernaryMotor::new(
        Motor::new(peripherals.port_17, Gearset::Green, Direction::Forward),
        MAX_WHEEL,
    );
    let outake = TernaryMotor::new(
        Motor::new(peripherals.port_16, Gearset::Green, Direction::Reverse),
        MAX_WHEEL,
    );

    let display = peripherals.display;

    let switch = AdiDigitalOut::with_initial_level(peripherals.adi_h, LogicLevel::High);
    let descore = AdiDigitalOut::with_initial_level(peripherals.adi_g, LogicLevel::Low);

    let robot = Robot {
        primary_controller: peripherals.primary_controller,
        drivetrain: RecordableDrivetrain::new(Drivetrain::new(
            Differential::new(left, right),
            NoTracking,
        )),
        intake,
        outake,
        switch,
        descore,
    };

    if cfg!(feature = "record") {
        RecordingAutonomous::compete(robot, display).await;
    } else {
        PlaybackAutonomous::compete(robot, display).await;
    }
}
