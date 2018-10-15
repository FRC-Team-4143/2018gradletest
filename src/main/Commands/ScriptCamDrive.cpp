#include "Commands/ScriptCamDrive.h"
#include "Modules/Logger.h"
#include "Robot.h"
#include <iostream>

// ==========================================================================

ScriptCamDrive::ScriptCamDrive(std::string name, double x, double y, double maxSpeed, double timeoutSeconds, int side)
: frc::Command(name), _visionSink(), _visionSource(), _pid(nullptr),
		_x(x), _y(y), _maxSpeed(maxSpeed), _timeoutSeconds(timeoutSeconds), _side(side),
		_counter(0), _angle(0), _p(0), _i(0), _d(0), _tol(0), _center(0), _returnQuick(false) {
	char szParams[64];
	sprintf(szParams, "(%f, %f, %f, %f, %i)", x, y, maxSpeed, timeoutSeconds, side);
	LOG(GetName() + "::ctor" + szParams);

	Requires(Robot::driveTrain);
	_pid = new PIDController(0, 0, 0, 0, &_visionSource, &_visionSink);
}

// ==========================================================================

void ScriptCamDrive::Initialize() {
	LOG(GetName() + "::Initialize");

	if (_timeoutSeconds == 0) {
		SetTimeout(16);
		_returnQuick = true;
	} else {
		SetTimeout(_timeoutSeconds);
		_returnQuick = false;
	}

	_counter = 0;
	_angle = Robot::gyroSub->PIDGet();
	_angle /= 90;
	_angle = floor(_angle + .5); // round
	_angle *= 90;
	_p = SmartDashboard::GetNumber("vision P", .3);
	_i = SmartDashboard::GetNumber("vision I", .3);
	_d = SmartDashboard::GetNumber("vision D", .3);
	_pid->SetPID(_p / 100, _i / 100, _d / 100);
	_tol = SmartDashboard::GetNumber("vision tol", 15);
	_center = SmartDashboard::GetNumber("vision center", 25.0);
	_pid->SetAbsoluteTolerance(_tol);
	_pid->SetSetpoint(_center);
	_pid->Enable();
	_pid->SetOutputRange(-0.25, 0.25);
	Robot::driveTrain->enableSpeedControl();
}

// ==========================================================================

void ScriptCamDrive::Execute() {
	auto offset = Robot::visionBridge->GetBoilerPosition();
	if (std::abs(offset - _center) < _tol) {
		_counter++;
	} else {
		_counter = 0;
	}

	double speed = -_visionSink.GetValue();
	if (speed < 0.01 && speed > -0.01) {
		speed = 0.01;
	}
	SmartDashboard::PutNumber("vision pid value", speed);
	Robot::driveTrain->Crab(speed, _x, _y, false);
}

// ==========================================================================

bool ScriptCamDrive::IsFinished() {
	return IsTimedOut() || (_returnQuick && _counter > 10);
}

// ==========================================================================

void ScriptCamDrive::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void ScriptCamDrive::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void ScriptCamDrive::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
	_pid->Disable();
	Robot::driveTrain->disableSpeedControl();
	_counter = 0;
}

// ==========================================================================
