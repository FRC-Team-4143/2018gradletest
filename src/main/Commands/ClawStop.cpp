#include "Commands/ClawStop.h"
#include "Robot.h"

ClawStop::ClawStop() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::claw);
}

void ClawStop::Initialize() {
}

void ClawStop::Execute() {
		Robot::claw->clawStop();
}

bool ClawStop::IsFinished() {
	return false;
}

void ClawStop::End() {
}

void ClawStop::Interrupted() {
	End();
}
