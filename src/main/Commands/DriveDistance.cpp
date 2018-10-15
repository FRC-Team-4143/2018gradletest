#include "Commands/DriveDistance.h"
#include "Robot.h"

// ==========================================================================

DriveDistance::DriveDistance(float twistAngle, float distance, float timeout)
: _twistAngle(twistAngle), _distance(distance), _timeout(timeout), _count(0), _fCount(0) {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void DriveDistance::Initialize() {
	_twistAngle *= Robot::inverter;
	_count = 0;
	_fCount = 0;
	Robot::driveTrain->DriveDistanceStart(_twistAngle, _distance);
	if (_timeout != 0) {
		SetTimeout(_timeout);
	}
}

// ==========================================================================

void DriveDistance::Execute() {
	Robot::driveTrain->DriveDistanceExecute(_twistAngle, _distance);
	_count++;
}

// ==========================================================================

bool DriveDistance::IsFinished() {
	Robot::driveTrain->updateDistanceEncoders();
	if (IsTimedOut() || (Robot::driveTrain->WheelVelocityZero()
		&& Robot::driveTrain->getDistanceEncodersValues() > 0.5 && _count > 25 && (Robot::driveTrain->getDistanceEncodersValues() >= (_distance - 2.0)))) {
		_fCount++;
	}
	else {
		_fCount = 0;
	}
	return _fCount > 1;
}

// ==========================================================================

void DriveDistance::End() {
	Robot::driveTrain->DriveDistanceEnd();
}

// ==========================================================================

void DriveDistance::Interrupted() {
	End();
}

// ==========================================================================
