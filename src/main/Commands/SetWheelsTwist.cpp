#include "Commands/SetWheelsTwist.h"
#include "Robot.h"

// ==========================================================================

SetWheelsTwist::SetWheelsTwist(double seconds)
: _seconds(seconds) {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void SetWheelsTwist::Initialize() {
	SetTimeout(_seconds);
	Robot::driveTrain->PositionModeTwist(0);
}

// ==========================================================================

void SetWheelsTwist::Execute() {
}

// ==========================================================================

bool SetWheelsTwist::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void SetWheelsTwist::End() {
}

// ==========================================================================

void SetWheelsTwist::Interrupted() {
	End();
}

// ==========================================================================
