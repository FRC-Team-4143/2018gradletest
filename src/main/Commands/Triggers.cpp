#include "Commands/Triggers.h"
#include "Robot.h"
#include "OI.h"

Triggers::Triggers() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::elevator);
}

void Triggers::Initialize() {
}

void Triggers::Execute() {
	if (Robot::oi->GetRightTrigger() > 0) {
		Robot::elevator->elevatorUp(Robot::oi->GetRightTrigger()*1);
	}
	else if (Robot::oi->GetLeftTrigger() > 0) {
		Robot::elevator->elevatorDown(Robot::oi->GetLeftTrigger() * 0.7);
	}
	else {
		Robot::elevator->elevatorStop();
	}
}

bool Triggers::IsFinished() {
	return false;
}

void Triggers::End() {
	Robot::elevator->elevatorStop();
}

void Triggers::Interrupted() {
	End();
}
