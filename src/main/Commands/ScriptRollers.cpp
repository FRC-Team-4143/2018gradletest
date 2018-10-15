#include "Commands/ScriptRollers.h"
#include "Robot.h"

// ==========================================================================

ScriptRollers::ScriptRollers(float power, float timeout)
:_power(power), _timeout(timeout) {

	Requires(Robot::roller);
}

// ==========================================================================

void ScriptRollers::Initialize() {
	SetTimeout(_timeout);
}

// ==========================================================================

void ScriptRollers::Execute() {
	if(_power > 0)Robot::roller->rollerIn(_power);
	else Robot::roller->rollerOut(- _power);

}

// ==========================================================================

bool ScriptRollers::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptRollers::End() {
	Robot::roller->rollerStop();
}

// ==========================================================================

void ScriptRollers::Interrupted() {
	End();
}

// ==========================================================================
