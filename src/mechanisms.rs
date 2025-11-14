use vexide::{devices::smart::motor::MotorError, prelude::*};

pub struct Intake {
    pub intake: Motor,
    pub outtake: Motor,
}

impl Intake {
    pub fn forward(&mut self) -> Result<(), MotorError> {
        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE)?;
        self.outtake.set_voltage(Motor::V5_MAX_VOLTAGE)?;

        Ok(())
    }

    pub fn reverse(&mut self) -> Result<(), MotorError> {
        self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE)?;
        self.outtake.set_voltage(-Motor::V5_MAX_VOLTAGE)?;

        Ok(())
    }

    pub fn disable(&mut self) -> Result<(), MotorError> {
        self.intake.set_voltage(0.)?;
        self.outtake.set_voltage(0.)?;

        Ok(())
    }
}

pub struct Router {
    pub router: Motor,
}

impl Router {
    pub fn forward(&mut self) -> Result<(), MotorError> {
        self.router.set_voltage(Motor::V5_MAX_VOLTAGE)?;

        Ok(())
    }

    pub fn reverse(&mut self) -> Result<(), MotorError> {
        self.router.set_voltage(-Motor::V5_MAX_VOLTAGE)?;

        Ok(())
    }

    pub fn disable(&mut self) -> Result<(), MotorError> {
        self.router.set_voltage(0.)?;

        Ok(())
    }
}
