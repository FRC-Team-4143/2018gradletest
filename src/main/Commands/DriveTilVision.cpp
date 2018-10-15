#include "Commands/DriveTilVision.h"
#include "Robot.h"

// ==========================================================================

DriveTilVision::DriveTilVision(double y, double x, double twist, double seconds)
: _x(x), _y(y), _twist(twist), _seconds(seconds) {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void DriveTilVision::Initialize() {
	SetTimeout(_seconds);
}

// ==========================================================================

void DriveTilVision::Execute() {
	Robot::driveTrain->Crab(_twist, _y, _x, false);
}

// ==========================================================================

bool DriveTilVision::IsFinished() {
	return (Robot::visionBridge->GetGearDistance() != 0 &&
		Robot::visionBridge->GetGearDistance() < 100) ||
		IsTimedOut();
}

// ==========================================================================

void DriveTilVision::End() {
	Robot::driveTrain->Crab(0, 0, 0, false);
}

// ==========================================================================

void DriveTilVision::Interrupted() {
	End();
}

// ==========================================================================
