#include "Commands/GyroCrab.h"
#include "Modules/Logger.h"
#include "Robot.h"
#include <cmath>

// ==========================================================================

GyroCrab::GyroCrab()
: frc::Command("Crab Drive"), _angle(0) {
	LOG (GetName() + "::ctor");

	Requires(Robot::driveTrain);
	flag = false;
}

// ==========================================================================

void GyroCrab::Initialize() {
	_angle = Robot::gyroSub->PIDGet();
	_angle /= 90;
	_angle = floor(_angle + .5); // round
	_angle *= 90;

}

// ==========================================================================

void GyroCrab::Execute() {
	auto x = Robot::oi->GetJoystickX();
	auto y = Robot::oi->GetJoystickY();

	auto twist = Robot::oi->GetJoystickRX();

	_angle += twist * 1.0; //TODO scale later

	Robot::driveTrain->GyroCrab(_angle, -y, x, true);
}

// ==========================================================================

bool GyroCrab::IsFinished() {
	return false;
}

// ==========================================================================

void GyroCrab::End() {
}

// ==========================================================================

void GyroCrab::Interrupted() {
	End();
}

// ==========================================================================
