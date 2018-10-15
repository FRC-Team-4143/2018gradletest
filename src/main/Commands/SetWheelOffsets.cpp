#include "Commands/SetWheelOffsets.h"
#include "Robot.h"

// ==========================================================================

SetWheelOffsets::SetWheelOffsets() {
	SetRunWhenDisabled(true);
}

// ==========================================================================

void SetWheelOffsets::Initialize() {
	SetTimeout(1);
	Robot::driveTrain->setWheelOffsets();
}

// ==========================================================================

void SetWheelOffsets::Execute() {
}

// ==========================================================================

bool SetWheelOffsets::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void SetWheelOffsets::End() {
}

// ==========================================================================

void SetWheelOffsets::Interrupted() {
	End();
}

// ==========================================================================
