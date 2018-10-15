#include "Commands/RollerIn.h"
#include "Robot.h"

RollerIn::RollerIn() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::roller);
}

void RollerIn::Initialize() {
	count = 0;
}

void RollerIn::Execute() {
	count++;
	if(count >= 3) Robot::roller->rollerIn(1.0);
	}

bool RollerIn::IsFinished() {
	return false;
}

void RollerIn::End() {
	Robot::roller->rollerStop();
}

void RollerIn::Interrupted() {
	End();
}
