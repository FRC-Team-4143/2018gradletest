#include "Subsystems/Climber.h"
#include "RobotMap.h"
#include "ctre/phoenix.h"

Climber::Climber() : Subsystem("Climber") {
	climbingMotor = RobotMap::climbingMotor;

}

void Climber::InitDefaultCommand() {

}

void Climber::climb(float speed) {
	climbingMotor->Set(ControlMode::PercentOutput,-speed);

}

void Climber::reverseClimb(float speed) {
	climbingMotor->Set(ControlMode::PercentOutput,speed);
}

void Climber::stopClimb() {
	climbingMotor->Set(ControlMode::PercentOutput,0);

}
