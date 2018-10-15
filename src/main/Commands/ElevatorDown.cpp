#include "Commands/ElevatorDown.h"
#include "Robot.h"

ElevatorDown::ElevatorDown() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::elevator);
}

void ElevatorDown::Initialize() {
}

void ElevatorDown::Execute() {
	Robot::elevator->elevatorDown(0.4);
}

bool ElevatorDown::IsFinished() {
	return false;
}

void ElevatorDown::End() {
	Robot::elevator->elevatorStop();
}

void ElevatorDown::Interrupted() {
	End();
}
