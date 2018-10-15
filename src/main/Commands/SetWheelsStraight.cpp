#include "Commands/SetWheelsStraight.h"
#include "Robot.h"

SetWheelsStraight::SetWheelsStraight() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::driveTrain);
}

void SetWheelsStraight::Initialize() {
}

void SetWheelsStraight::Execute() {
	Robot::driveTrain->SetWheelsStraight();
}

bool SetWheelsStraight::IsFinished() {
	return false;
}

void SetWheelsStraight::End() {

}

void SetWheelsStraight::Interrupted() {
	End();
}
