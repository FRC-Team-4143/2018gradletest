#include "Commands/ScriptFieldCentricCrab.h"
#include "Robot.h"

// ==========================================================================

ScriptFieldCentricCrab::ScriptFieldCentricCrab(float twist, float y, float x, float seconds)
: _twist(twist), _y(y), _x(x), _seconds(seconds) {

	Requires(Robot::driveTrain);
}

// ==========================================================================

void ScriptFieldCentricCrab::Initialize() {
	SetTimeout(_seconds);
}

// ==========================================================================

void ScriptFieldCentricCrab::Execute() {
	Robot::driveTrain->FieldCentricCrab(_twist, _y, _x, false);
}

// ==========================================================================

bool ScriptFieldCentricCrab::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptFieldCentricCrab::End() {
	Robot::driveTrain->Crab(0, 0, 0, false);
}

// ==========================================================================

void ScriptFieldCentricCrab::Interrupted() {
	End();
}

// ==========================================================================
