#include "Commands/ScriptGyroDrive.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptGyroDrive::ScriptGyroDrive(std::string name, float y, float x, float z, float seconds)
: frc::Command(name), _x(x), _y(y), _z(z), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f, %f, %f, %f)", x, y, z, seconds);
	LOG(GetName() + "::ctor" + szParams);

	Requires(Robot::driveTrain);
}

// ==========================================================================

void ScriptGyroDrive::Initialize() {
	LOG(GetName() + "::Initialize");
	_z *= Robot::inverter;
	SetTimeout(_seconds);
	Robot::driveTrain->enableSpeedControl();
}

// ==========================================================================

void ScriptGyroDrive::Execute() {
	Robot::driveTrain->GyroCrab(_z, _y, _x, false);
}

// ==========================================================================

bool ScriptGyroDrive::IsFinished() { return IsTimedOut(); }

// ==========================================================================

void ScriptGyroDrive::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void ScriptGyroDrive::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void ScriptGyroDrive::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
	Robot::driveTrain->disableSpeedControl();
}

// ==========================================================================
