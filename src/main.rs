#![no_main]
#![no_std]

use autons::{
    prelude::{SelectCompete, SelectCompeteExt},
    route,
    simple::SimpleSelect,
};
use evian::{drivetrain::model::Mecanum, prelude::*};
use vexide::prelude::*;

use crate::{
    auton::FRAMES,
    mechanisms::{Intake, Router},
    teams::*,
};

/*const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);
const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(4.0)
    .velocity(0.25)
    .duration(Duration::from_millis(15));
const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(f64::to_radians(8.0))
    .velocity(0.09)
    .duration(Duration::from_millis(15));*/

extern crate alloc;

mod auton;
mod mechanisms;
mod teams;

pub const MAX_AUTO: f64 = Motor::V5_MAX_VOLTAGE * 0.4;

pub const INCH_TO_METER: f64 = 0.0254;
pub const BALL_DIAMETER: f64 = 3.25 * INCH_TO_METER;

pub const WHEEL_DIAMETER: f64 = 4.;
const TRACK_WIDTH: f64 = 14.;

// To rotate the body of the robot N degrees, spin the left/right wheels by ROBOT_TO_WHEEL_ROT * N,
// and the opposite side by -ROBOT_TO_WHEEL_ROT * N degrees. Swap which wheels get the negative to
// change turning direction. This works for both radians and degrees, the input and output are
// consistent with units.
// const ROBOT_TO_WHEEL_ROT: f64 = 2100. / 360.0; // TRACK_WIDTH / WHEEL_DIAMETER;

pub struct Robot {
    controller: Controller,

    intake: Intake,
    router: Router,

    drivetrain: Mecanum, // Drivetrain<Mecanum, GpsWheeledTracking>,
}

impl Robot {
    async fn autonomous(&mut self, alliance: Alliance, side: Side) {
        if side == Side::Left {
            return;
        }

        for frame in FRAMES {
            self.drivetrain
                .drive_vector(
                    Vec2 {
                        x: frame.x,
                        y: frame.y,
                    },
                    frame.r,
                )
                .ok();

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }

    async fn route_red_left(&mut self) {
        self.autonomous(Alliance::Red, Side::Left).await
    }

    async fn route_red_right(&mut self) {
        self.autonomous(Alliance::Red, Side::Right).await
    }

    async fn route_blue_left(&mut self) {
        self.autonomous(Alliance::Blue, Side::Left).await
    }

    async fn route_blue_right(&mut self) {
        self.autonomous(Alliance::Blue, Side::Right).await
    }
}

impl SelectCompete for Robot {
    async fn driver(&mut self) {
        loop {
            let controller_state = self.controller.state().unwrap_or_default();

            // Forward/Back
            let lx = controller_state.right_stick.x();

            // Strafe
            let ly = controller_state.right_stick.y();

            // Rotation
            let rx = controller_state.left_stick.x();

            // println!("[{:?}] {lx}\t{ly}\t{rx}", Instant::now());

            self.drivetrain.drive_vector(Vec2 { x: lx, y: ly }, rx).ok();

            let int_fw = controller_state.button_r1.is_pressed();
            let int_bw = controller_state.button_l1.is_pressed();

            let rou_fw = controller_state.button_r2.is_pressed();
            let rou_bw = controller_state.button_l2.is_pressed();

            if int_fw && !int_bw {
                println!("IntakeActivate");
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

    /*let gps = GpsSensor::new(
        peripherals.port_19,
        Point2 { x: 0., y: -0.08 },
        Point2 { x: 1.41, y: -0.7 },
        270.,
    );*/

    let front_left_motors = shared_motors![left_front];
    let back_left_motors = shared_motors![left_back];
    let front_right_motors = shared_motors![right_front];
    let back_right_motors = shared_motors![right_back];

    let drivetrain = Mecanum {
        front_left_motors: front_left_motors.clone(),
        back_left_motors: back_left_motors.clone(),
        front_right_motors: front_right_motors.clone(),
        back_right_motors: back_right_motors.clone(),
    };

    /*let drivetrain = Drivetrain::new(
        drivetrain,
        GpsWheeledTracking::new(
            gps,
            [
                TrackingWheel::new(front_left_motors, WHEEL_DIAMETER, -TRACK_WIDTH, None),
                TrackingWheel::new(back_left_motors, WHEEL_DIAMETER, -TRACK_WIDTH, None),
                TrackingWheel::new(front_right_motors, WHEEL_DIAMETER, TRACK_WIDTH, None),
                TrackingWheel::new(back_right_motors, WHEEL_DIAMETER, TRACK_WIDTH, None),
            ],
        ),
    );*/

    let robot = Robot {
        controller,

        intake: Intake { intake, outtake },
        router: Router { router },

        drivetrain,
    };

    robot
        .compete(SimpleSelect::new(
            peripherals.display,
            [
                // route!("Red, Left (NON-FUNCTIONAL)", Robot::route_red_left),
                route!("Red, Right", Robot::route_red_right),
                // route!("Blue, Left (NON-FUNCTIONAL)", Robot::route_blue_left),
                route!("Blue, Right", Robot::route_blue_right),
            ],
        ))
        .await;
}
