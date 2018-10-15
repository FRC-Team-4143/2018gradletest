#include "Commands/UnwindWheels.h"
#include "Robot.h"

// ==========================================================================

UnwindWheels::UnwindWheels() {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void UnwindWheels::Initialize() {
	SmartDashboard::PutString("Unwinding", "true");
}

// ==========================================================================

void UnwindWheels::Execute() {
	Robot::driveTrain->unwind();
}

// ==========================================================================

bool UnwindWheels::IsFinished() {
	return false;
}

// ==========================================================================

void UnwindWheels::End() {
	SmartDashboard::PutString("Unwinding", "false");
	Robot::driveTrain->doneunwind();
}

// ==========================================================================

void UnwindWheels::Interrupted() {
	End();
}

// ==========================================================================
