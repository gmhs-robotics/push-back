#![no_main]
#![no_std]

use core::time::Duration;

use autons::{
    prelude::{SelectCompete, SelectCompeteExt},
    route,
    simple::SimpleSelect,
};
use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::Mecanum,
    motion::Basic,
    prelude::*,
};
use vexide::{
    devices::{math::Point2, smart::GpsSensor},
    prelude::*,
};

use crate::{
    gps::GpsTracking,
    mechanisms::{Intake, Router},
    teams::*,
};

const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);
const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(4.0)
    .velocity(0.25)
    .duration(Duration::from_millis(15));
const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(f64::to_radians(8.0))
    .velocity(0.09)
    .duration(Duration::from_millis(15));

extern crate alloc;

mod gps;
mod mechanisms;
mod teams;

pub const MAX_AUTO: f64 = Motor::V5_MAX_VOLTAGE * 0.4;

pub const INCH_TO_METER: f64 = 0.0254;
pub const BALL_DIAMETER: f64 = 3.25 * INCH_TO_METER;
pub const WHEEL_DIAMETER: f64 = 4. * INCH_TO_METER;
// const TRACK_WIDTH: f64 = 14. * INCH_TO_METER;

// To rotate the body of the robot N degrees, spin the left/right wheels by ROBOT_TO_WHEEL_ROT * N,
// and the opposite side by -ROBOT_TO_WHEEL_ROT * N degrees. Swap which wheels get the negative to
// change turning direction. This works for both radians and degrees, the input and output are
// consistent with units.
// const ROBOT_TO_WHEEL_ROT: f64 = 2100. / 360.0; // TRACK_WIDTH / WHEEL_DIAMETER;

pub struct Robot {
    controller: Controller,

    intake: Intake,
    router: Router,

    drivetrain: Drivetrain<Mecanum, GpsTracking>,
}

impl Robot {
    async fn autonomous(&mut self, alliance: Alliance, side: Side) {
        if side == Side::Left {
            return;
        }

        let dt = &mut self.drivetrain;

        let mut basic = Basic {
            linear_controller: LINEAR_PID,
            angular_controller: ANGULAR_PID,
            linear_tolerances: LINEAR_TOLERANCES,
            angular_tolerances: ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(10)),
        };

        self.intake.forward().ok();
        basic.drive_distance(dt, 13. * BALL_DIAMETER).await; // TODO: Adjust
        self.intake.disable().ok();

        let tube_rot = Angle::from_degrees(match alliance {
            Alliance::Red => 45.,
            Alliance::Blue => 225.,
        });

        basic
            .drive_distance_at_heading(dt, 4. * BALL_DIAMETER, tube_rot)
            .await;

        self.intake.reverse().ok();
        sleep(Duration::from_secs(3)).await;
        self.intake.disable().ok();

        basic.drive_distance(dt, -4. * BALL_DIAMETER).await;
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

            self.drivetrain
                .model
                .drive_vector(Vec2 { x: lx, y: ly }, rx)
                .ok();

            /*let v_fl = ly - lx + rx;
            let v_fr = ly + lx - rx;
            let v_bl = ly + lx + rx;
            let v_br = ly - lx - rx;

            self.left_front.set_voltage(v_fl * MAX_WHEEL).ok();
            self.left_back.set_voltage(v_bl * MAX_WHEEL).ok();
            self.right_front.set_voltage(v_fr * MAX_WHEEL).ok();
            self.right_back.set_voltage(v_br * MAX_WHEEL).ok();*/

            let int_fw = controller_state.button_r1.is_pressed();
            let int_bw = controller_state.button_l1.is_pressed();

            let rou_fw = controller_state.button_r2.is_pressed();
            let rou_bw = controller_state.button_l2.is_pressed();

            if int_fw && !int_bw {
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

            self.drivetrain.tracking.integrate_acceleration();

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

    let gps = GpsSensor::new(
        peripherals.port_19,
        Point2 { x: 0., y: -0.08 },
        Point2 { x: 1.41, y: -0.7 },
        270.,
    );

    let drivetrain = Drivetrain::new(
        Mecanum {
            front_left_motors: shared_motors![left_front],
            back_left_motors: shared_motors![left_back],
            front_right_motors: shared_motors![right_front],
            back_right_motors: shared_motors![right_back],
        },
        GpsTracking::new(gps),
    );

    let robot = Robot {
        controller,

        intake: Intake { intake, outtake },
        router: Router { router },

        // gps,
        drivetrain,
        /*left_front,
        left_back,
        right_front,
        right_back,*/
    };

    robot
        .compete(SimpleSelect::new(
            peripherals.display,
            [
                route!("Red, Left (NON-FUNCTIONAL)", Robot::route_red_left),
                route!("Red, Right", Robot::route_red_right),
                route!("Blue, Left (NON-FUNCTIONAL)", Robot::route_blue_left),
                route!("Blue, Right", Robot::route_blue_right),
            ],
        ))
        .await;
}
