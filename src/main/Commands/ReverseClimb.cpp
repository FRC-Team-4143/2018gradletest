#include "Commands/ReverseClimb.h"
#include "Robot.h"

ReverseClimb::ReverseClimb() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::climber);
}

void ReverseClimb::Initialize() {
}

void ReverseClimb::Execute() {

}

bool ReverseClimb::IsFinished() {
	return false;
}

void ReverseClimb::End() {

}

void ReverseClimb::Interrupted() {
	End();
}
