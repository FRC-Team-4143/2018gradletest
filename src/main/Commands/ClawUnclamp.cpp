#include "Commands/ClawUnclamp.h"
#include "Robot.h"

ClawUnclamp::ClawUnclamp() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::claw);
}

void ClawUnclamp::Initialize() {
}

void ClawUnclamp::Execute() {
	Robot::claw->clawUnclamp();
}
bool ClawUnclamp::IsFinished() {
	return false;
}

void ClawUnclamp::End() {
	Robot::claw->clawStop();
}

void ClawUnclamp::Interrupted() {
	End();
}
