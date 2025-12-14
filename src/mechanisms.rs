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
}
