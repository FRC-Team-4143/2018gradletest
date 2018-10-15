#include "Subsystems/Roller.h"
#include "RobotMap.h"
#include "ctre/phoenix.h"
#include "Robot.h"
#include "OI.h"


Roller::Roller() : Subsystem("Roller") {
	rollerMotor = RobotMap::rollerMotor;
}

void Roller::InitDefaultCommand() {
}

void Roller::rollerOut(float speed) {
	rollerMotor->Set(ControlMode::PercentOutput, speed);
}

void Roller::rollerIn(float speed) {
	rollerMotor->Set(ControlMode::PercentOutput, -speed);
}

void Roller::rollerStop() {
	rollerMotor->Set(ControlMode::PercentOutput, 0);
}
