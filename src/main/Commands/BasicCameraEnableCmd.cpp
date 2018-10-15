#include "Commands/BasicCameraEnableCmd.h"

// ==========================================================================

BasicCameraEnableCmd::BasicCameraEnableCmd(std::shared_ptr<BasicCameraSub> camera)
: frc::Command("BasicCameraEnable"), m_camera(camera) {
	//Requires(camera.get());
	SetRunWhenDisabled(true);
}

// ==========================================================================
// Called just before this Command runs the first time

void BasicCameraEnableCmd::Initialize() {
}

// ==========================================================================
// Called repeatedly when this Command is scheduled to run

void BasicCameraEnableCmd::Execute() {
	m_camera->Enable();
}

// ==========================================================================
// Make this return true when this Command no longer needs to run Execute.

bool BasicCameraEnableCmd::IsFinished() {
	return true;
}

// ==========================================================================
// Called once after isFinished returns true

void BasicCameraEnableCmd::End() {
}

// ==========================================================================
// Called when another command which requires this subsystem is scheduled to run

void BasicCameraEnableCmd::Interrupted() {
	End();
}

// ==========================================================================
