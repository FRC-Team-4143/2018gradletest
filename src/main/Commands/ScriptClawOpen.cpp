#include "Commands/ScriptClawOpen.h"
#include "Robot.h"

// ==========================================================================

ScriptClawOpen::ScriptClawOpen(float timeout)
: _timeout(timeout) {

	Requires(Robot::claw);
}

// ==========================================================================

void ScriptClawOpen::Initialize() {
	SetTimeout(_timeout);
}

// ==========================================================================

void ScriptClawOpen::Execute() {
	Robot::claw->clawUnclamp();
}

// ==========================================================================

bool ScriptClawOpen::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptClawOpen::End() {
}

// ==========================================================================

void ScriptClawOpen::Interrupted() {
	End();
}

// ==========================================================================
