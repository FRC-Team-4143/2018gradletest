#include "Commands/ClawClamp.h"
#include "Robot.h"

ClawClamp::ClawClamp() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::claw);
}

void ClawClamp::Initialize() {
}

void ClawClamp::Execute() {
		Robot::claw->clawClamp(0.1);
}

bool ClawClamp::IsFinished() {
	return false;
}

void ClawClamp::End() {
	Robot::claw->clawStop();
}

void ClawClamp::Interrupted() {
	End();
}
