use vexide::{prelude::*, smart::PortError};

pub struct BinaryMotor(pub Motor, pub f64);

impl BinaryMotor {
    pub fn disable(&mut self) -> Result<(), PortError> {
        self.0.set_voltage(0.)
    }

    pub fn forward(&mut self) -> Result<(), PortError> {
        self.0.set_voltage(self.1)
    }

    pub fn reverse(&mut self) -> Result<(), PortError> {
        self.0.set_voltage(-self.1)
    }

    pub fn update_from_button_state(
        &mut self,
        forward: bool,
        reverse: bool,
    ) -> Result<(), MotorError> {
        if forward && !reverse {
            self.forward()
        } else if reverse && !forward {
            self.reverse()
        } else {
            self.disable()
        }
    }
}
