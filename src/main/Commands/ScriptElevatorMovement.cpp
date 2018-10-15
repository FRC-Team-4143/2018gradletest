#include "Commands/ScriptElevatorMovement.h"
#include "Robot.h"

// ==========================================================================

ScriptElevatorMovement::ScriptElevatorMovement(float power, float height, float timeout)
: _power(power), _height(height),_timeout(timeout) {

	Requires(Robot::elevator);
}

// ==========================================================================

void ScriptElevatorMovement::Initialize() {
	SetTimeout(_timeout);

}

// ==========================================================================

void ScriptElevatorMovement::Execute() {
	if((_height - Robot::elevator->getElevatorPosition()) > 0){
			Robot::elevator->elevatorUp(_power);
		}

	else {
		if((_height - Robot::elevator->getElevatorPosition()) < 0){
					Robot::elevator->elevatorDown(_power);
			}
	}
}

// ==========================================================================

bool ScriptElevatorMovement::IsFinished() {
	if(abs(_height - Robot::elevator->getElevatorPosition()) < 0.01){
		return true;
	}
	return IsTimedOut();
}

// ==========================================================================

void ScriptElevatorMovement::End() {
	Robot::elevator->elevatorStop();
}

// ==========================================================================

void ScriptElevatorMovement::Interrupted() {
	End();
}

// ==========================================================================
