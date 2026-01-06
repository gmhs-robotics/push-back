use std::ops::Not;

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
            0.
        }
    }

    pub fn calculate_from_ternary(&self, ternary: Ternary) -> f64 {
        match ternary {
            Ternary::High => self.baseline_voltage,
            Ternary::Zero => 0.,
            Ternary::Low => -self.baseline_voltage,
        }
    }
}

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
