#include "Commands/RollerOut.h"
#include "Robot.h"

RollerOut::RollerOut() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::roller);
	//Requires(Robot::claw);
}

void RollerOut::Initialize() {
}

void RollerOut::Execute() {
	Robot::roller->rollerOut(0.75);
	//Robot::claw->clawStop();
}

bool RollerOut::IsFinished() {
	return false;
}

void RollerOut::End() {
	Robot::roller->rollerStop();
}

void RollerOut::Interrupted() {
	End();
}
