use vexide::{devices::smart::motor::MotorError, prelude::*};

pub struct ControlledMotorGroup<const N: usize> {
    target_voltage: f64,
    motors: [Motor; N],
}

impl<const N: usize> ControlledMotorGroup<N> {
    pub fn new(target_voltage: f64, motors: [Motor; N]) -> Self {
        Self {
            target_voltage,
            motors,
        }
    }

    pub fn set_voltage(&mut self, voltage: f64) -> Result<(), MotorError> {
        for motor in &mut self.motors {
            motor.set_voltage(voltage)?;
        }

        Ok(())
    }

    pub fn forward(&mut self) -> Result<(), MotorError> {
        self.set_voltage(self.target_voltage)
    }

    pub fn reverse(&mut self) -> Result<(), MotorError> {
        self.set_voltage(-self.target_voltage)
    }

    pub fn disable(&mut self) -> Result<(), MotorError> {
        self.set_voltage(0.)
    }

    pub fn drive_by_buttons(&mut self, forward: bool, reverse: bool) -> Result<(), MotorError> {
        if forward && !reverse {
            self.forward()
        } else if reverse && !forward {
            self.reverse()
        } else {
            self.disable()
        }
    }
}
