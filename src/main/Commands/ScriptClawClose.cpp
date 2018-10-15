#include "Commands/ScriptClawClose.h"
#include "Robot.h"

// ==========================================================================

ScriptClawClose::ScriptClawClose(float timeout)
:_timeout(timeout) {

	Requires(Robot::claw);
}

// ==========================================================================

void ScriptClawClose::Initialize() {
	SetTimeout(_timeout);
}

// ==========================================================================

void ScriptClawClose::Execute() {
	Robot::claw->clawClamp(0.2);
}

// ==========================================================================

bool ScriptClawClose::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptClawClose::End() {
}

// ==========================================================================

void ScriptClawClose::Interrupted() {
	End();
}

// ==========================================================================
