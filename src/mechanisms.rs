use std::ops::Not;

use ozton::record::{RecordField, frame::RecordMode};
use vexide::{prelude::*, smart::PortError};

pub struct TernaryMotor {
    pub motor: Motor,
    baseline_voltage: f64,
}

impl TernaryMotor {
    pub fn new(motor: Motor, baseline_voltage: f64) -> Self {
        Self {
            motor,
            baseline_voltage,
        }
    }

    #[allow(dead_code)]
    pub fn update_from_button_state(
        &mut self,
        forward: bool,
        reverse: bool,
    ) -> Result<f64, PortError> {
        let volts = self.calculate_from_button_state(forward, reverse);

        self.motor.set_voltage(volts)?;

        Ok(volts)
    }

    pub fn calculate_from_button_state(&self, forward: bool, reverse: bool) -> f64 {
        if forward && !reverse {
            self.baseline_voltage
        } else if reverse && !forward {
            -self.baseline_voltage
        } else {
            0.0
        }
    }

    pub fn calculate_from_ternary(&self, ternary: Ternary) -> f64 {
        match ternary {
            Ternary::High => self.baseline_voltage,
            Ternary::Zero => 0.0,
            Ternary::Low => -self.baseline_voltage,
        }
    }
}

#[ozton::record::async_trait(?Send)]
impl RecordField for TernaryMotor {
    type Output = f64;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        self.motor.set_voltage(*frame)
    }

    async fn stop_playback(&mut self) -> Result<(), PortError> {
        self.motor.set_voltage(0.0)
    }
}

#[derive(PartialEq)]
pub enum Ternary {
    High,
    Low,
    Zero,
}

impl From<bool> for Ternary {
    fn from(value: bool) -> Self {
        if value { Self::High } else { Self::Low }
    }
}

impl Not for Ternary {
    type Output = Self;

    fn not(self) -> Self::Output {
        match self {
            Self::High => Self::Low,
            Self::Low => Self::High,
            Self::Zero => Self::Zero,
        }
    }
}

pub fn adi_toggle_pure(adi: &AdiDigitalOut, toggle: bool) -> Result<bool, PortError> {
    let current_state = adi.is_high()?;

    let target_state = if toggle {
        !current_state
    } else {
        current_state
    };

    Ok(target_state)
}
