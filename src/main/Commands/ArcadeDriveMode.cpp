#include "Commands/ArcadeDriveMode.h"
#include "Robot.h"

// ==========================================================================

ArcadeDriveMode::ArcadeDriveMode() {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void ArcadeDriveMode::Initialize() {
	Robot::driveTrain->SetWheelsStraight();
}

// ==========================================================================

void ArcadeDriveMode::Execute() {
	Robot::driveTrain->ArcadeDriveMode(Robot::oi->GetJoystickY(), Robot::oi->GetJoystickX());
}

// ==========================================================================

bool ArcadeDriveMode::IsFinished() {
	return false;
}

// ==========================================================================

void ArcadeDriveMode::End() {
}

// ==========================================================================

void ArcadeDriveMode::Interrupted() {
	End();
}

// ==========================================================================
