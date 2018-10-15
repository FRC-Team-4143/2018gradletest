#include "Subsystems/Arm.h"
#include "RobotMap.h"
#include "ctre/phoenix.h"
#include "Robot.h"
#include "OI.h"

constexpr int UNITS_PER_ROTATION = 4096;
constexpr int NON_CLIMBING_MIN_ROTATIONS = 1000;

Arm::Arm() : Subsystem("Arm") {
	armMotor = RobotMap::armMotor;
}

void Arm::InitDefaultCommand() {
}

void Arm::armOut(float speed) {
	if(Robot::oi->IsInClimbMode()){
		armMotor->Set(ControlMode::PercentOutput, -0.5);
	}else{
		armMotor->Set(ControlMode::PercentOutput, -speed);
	}
}

void Arm::armIn(float speed) {
	if (Robot::elevator->IsInClimberRange() && !Robot::oi->IsInClimbMode()) {
		armMotor->Set(ControlMode::PercentOutput, 0);
	}
	else {
		armMotor->Set(ControlMode::PercentOutput, speed);
	}
}

void Arm::armStop() {
	armMotor->Set(ControlMode::PercentOutput, 0);
}

float Arm::getArmPosition() const {
	return armMotor->GetSelectedSensorPosition(0) /* UNITS_PER_ROTATION8*/;
}

bool Arm::IsInClimberRange() const {
	if(Robot::climberArrangement == 1) return false;
	else return getArmPosition() < NON_CLIMBING_MIN_ROTATIONS;
}
