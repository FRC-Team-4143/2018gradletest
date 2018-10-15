#include "Subsystems/Elevator.h"
#include "RobotMap.h"
#include "Robot.h"
#include "ctre/phoenix.h"
#include "Commands/Triggers.h"
#include "OI.h"

constexpr int UNITS_PER_FOOT = 4776;
constexpr float NON_CLIMBING_MIN_ROTATIONS = 0.3 ;

Elevator::Elevator() : Subsystem("Elevator") {
	elevatorMotor = RobotMap::elevatorMotor;
}

void Elevator::InitDefaultCommand() {
	SetDefaultCommand(new Triggers());
}

void Elevator::elevatorDown(float speed) {
	if (IsInClimberRange() && Robot::arm->IsInClimberRange() && !Robot::oi->IsInClimbMode()){
		elevatorMotor->Set(ControlMode::PercentOutput, 0);
	}else{
		elevatorMotor->Set(ControlMode::PercentOutput, -speed);
	}
}


void Elevator::elevatorUp(float speed) {
	if (IsInClimberRange() && Robot::arm->IsInClimberRange() && !Robot::oi->IsInClimbMode() ){
			elevatorMotor->Set(ControlMode::PercentOutput, 0);
		} else {
		elevatorMotor->Set(ControlMode::PercentOutput, speed);
	}
}

void Elevator::elevatorStop() {
	elevatorMotor->Set(ControlMode::PercentOutput, 0);
}

float Elevator::getElevatorPosition() const {
	return ((float) elevatorMotor->GetSelectedSensorPosition(0) /UNITS_PER_FOOT);
}

bool Elevator::IsInClimberRange() const {
	if(Robot::climberArrangement == 1)
		return false;
	else{
	return getElevatorPosition() < NON_CLIMBING_MIN_ROTATIONS;
	}
}
