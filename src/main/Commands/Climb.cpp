#include "Commands/Climb.h"
#include "Robot.h"

Climb::Climb() {
		Requires(Robot::climber);
}

void Climb::Initialize() {
}

void Climb::Execute() {
	if(Robot::oi->GetButtonX())
		Robot::climber->climb(-0.5);
	else{
		Robot::climber->climb(0.75);
	}
}

bool Climb::IsFinished() {
	return false;
}

void Climb::End() {
	Robot::climber->stopClimb();
}

void Climb::Interrupted() {
	End();
}
