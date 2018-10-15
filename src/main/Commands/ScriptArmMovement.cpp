#include "Commands/ScriptArmMovement.h"
#include "Robot.h"

// ==========================================================================

ScriptArmMovement::ScriptArmMovement(float power, float distance, float timeout)
: _power(power), _distance(distance),_timeout(timeout) {

	Requires(Robot::arm);
}

// ==========================================================================

void ScriptArmMovement::Initialize() {
	SetTimeout(_timeout);
}

// ==========================================================================

void ScriptArmMovement::Execute() {
	if((_distance - Robot::arm->getArmPosition()) > 0){
		Robot::arm->armOut(_power);
	}
	else {
		Robot::arm->armIn(_power);
	}
}

// ==========================================================================

bool ScriptArmMovement::IsFinished() {
	if(abs(_distance - Robot::arm->getArmPosition()) < 1000){
		return true;
	}
	return IsTimedOut();
}

// ==========================================================================

void ScriptArmMovement::End() {
	Robot::arm->armStop();
}

// ==========================================================================

void ScriptArmMovement::Interrupted() {
	End();
}

// ==========================================================================
