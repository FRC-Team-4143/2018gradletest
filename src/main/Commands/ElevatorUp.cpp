#include "Commands/ElevatorUp.h"
#include "Robot.h"

ElevatorUp::ElevatorUp() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::elevator);

	//RobotMap::elevatorMotor->GetSelectedSensorPosition();
}

void ElevatorUp::Initialize() {

}

void ElevatorUp::Execute() {

	Robot::elevator->elevatorUp(1.0);
}

bool ElevatorUp::IsFinished() {
	return false;
}

void ElevatorUp::End() {
	Robot::elevator->elevatorStop();
}

void ElevatorUp::Interrupted() {
	End();
}
