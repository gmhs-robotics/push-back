use std::time::{Duration, Instant};

use autons::{prelude::*, route, simple::SimpleSelect};
use bincode::config::{Configuration, LittleEndian, NoLimit, Varint};
use vexide::prelude::*;

use crate::{mechanisms::BinaryMotor, record::RecordedPath};

mod mechanisms;
mod record;

pub const MAX_WHEEL: f64 = Motor::V5_MAX_VOLTAGE;

struct Robot {
    controller: Controller,

    left: [Motor; 2],
    right: [Motor; 2],

    intake: BinaryMotor,
    outtake: BinaryMotor,

    piston: AdiDigitalOut,

    #[cfg(feature = "record")]
    recording: RecordedPath,
}

impl SelectCompete for Robot {
    async fn driver(&mut self) {
        #[cfg(feature = "record")]
        let mut last_frame = Instant::now();

        loop {
            #[cfg(feature = "record")]
            let now = Instant::now();
            #[cfg(feature = "record")]
            let delay: Duration = now.duration_since(last_frame);
            #[cfg(feature = "record")]
            {
                last_frame = now;
            }

            let controller_state = self.controller.state().unwrap_or_default();

            let t = controller_state.right_stick.y();
            let r = controller_state.left_stick.x();

            let left_volts = (t + r) * MAX_WHEEL;
            let right_volts = (t - r) * MAX_WHEEL;

            for motor in &mut self.left {
                motor.set_voltage(left_volts).ok();
            }

            for motor in &mut self.right {
                motor.set_voltage(right_volts).ok();
            }

            let intake_forward = controller_state.button_r2.is_pressed();
            let intake_reverse = controller_state.button_l2.is_pressed();

            let intake_volts = self
                .intake
                .update_from_button_state(intake_forward, intake_reverse)
                .unwrap_or_default();

            let outake_forward = controller_state.button_r1.is_pressed();
            let outake_reverse = controller_state.button_x.is_pressed();

            let outake_volts = self
                .outtake
                .update_from_button_state(outake_forward, outake_reverse)
                .unwrap_or_default();

            let toggle_piston = controller_state.button_l1.is_now_pressed();

            let piston_state = self.piston.is_high().unwrap_or_default();

            if toggle_piston {
                if piston_state {
                    let _ = self.piston.set_low();
                } else {
                    let _ = self.piston.set_high();
                }
            }

            #[cfg(feature = "record")]
            {
                use crate::record::Frame;

                self.recording.frames.push(Frame {
                    left: left_volts,
                    right: right_volts,
                    intake: intake_volts,
                    outake: outake_volts,
                    piston: if toggle_piston {
                        !piston_state
                    } else {
                        piston_state
                    },
                    delay: delay.as_micros() as u64,
                });
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

const BINCODE_CONFIG: Configuration<LittleEndian, Varint, NoLimit> = bincode::config::standard();

impl Robot {
    async fn save_recording(&mut self) {
        println!("SAVING");
        #[cfg(feature = "record")]
        {
            println!("REALLY");
            let route = bincode::encode_to_vec(&self.recording, BINCODE_CONFIG);

            if let Ok(bytes) = route {
                let _ = std::fs::write("left.route", bytes);
                println!("SAVED");
            }
        }
    }

    async fn route_none(&mut self) {}

    async fn route_recorded_left(&mut self) {
        println!("RUNNING");
        let route = std::fs::read("left.route");
        println!("parsed");

        if let Err(ref e) = route {
            println!("{e:?}");
        }

        if let Ok(route) = route {
            let route = bincode::decode_from_slice::<RecordedPath, _>(&route, BINCODE_CONFIG);
            println!("decoded");
            if let Ok((route, _bytes_read)) = route {
                println!("outt atime");
                for frame in route.frames {
                    for motor in &mut self.left {
                        let _ = motor.set_voltage(frame.left);
                    }

                    for motor in &mut self.right {
                        let _ = motor.set_voltage(frame.right);
                    }

                    let _ = self.intake.0.set_voltage(frame.intake);
                    let _ = self.outtake.0.set_voltage(frame.outake);

                    let _ = self.piston.set_level(if frame.piston {
                        vexide::adi::digital::LogicLevel::High
                    } else {
                        vexide::adi::digital::LogicLevel::Low
                    });

                    sleep(Duration::from_micros(frame.delay)).await;
                }
            }
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let controller = peripherals.primary_controller;

    let left = [
        Motor::new(peripherals.port_18, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_12, Gearset::Green, Direction::Forward),
    ];
    let right = [
        Motor::new(peripherals.port_10, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_9, Gearset::Green, Direction::Reverse),
    ];

    let intake = BinaryMotor(
        Motor::new(peripherals.port_19, Gearset::Green, Direction::Forward),
        MAX_WHEEL,
    );
    let outtake = BinaryMotor(
        Motor::new(peripherals.port_21, Gearset::Green, Direction::Forward),
        MAX_WHEEL,
    );

    let mut piston = AdiDigitalOut::new(peripherals.adi_a);

    let _ = piston.set_high();

    println!("WHATS UP");

    let robot = Robot {
        controller,

        left,
        right,
        intake,
        outtake,

        piston,

        #[cfg(feature = "record")]
        recording: RecordedPath { frames: vec![] },
    };

    robot
        .compete(SimpleSelect::new(
            peripherals.display,
            [
                route!("Disable", Robot::route_none),
                route!("Left (recorded)", Robot::route_recorded_left),
                route!("Save Recording", Robot::save_recording),
            ],
        ))
        .await;
}
