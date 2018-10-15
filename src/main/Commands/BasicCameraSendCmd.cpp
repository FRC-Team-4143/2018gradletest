#include "Commands/BasicCameraSendCmd.h"

// ==========================================================================

BasicCameraSendCmd::BasicCameraSendCmd(std::shared_ptr<BasicCameraSub> camera)
: frc::Command("BasicCameraSend"), m_camera(camera) {
	//Requires(camera.get());
	SetRunWhenDisabled(true);
}

// ==========================================================================
// Called just before this Command runs the first time

void BasicCameraSendCmd::Initialize() {
}

// ==========================================================================
// Called repeatedly when this Command is scheduled to run

void BasicCameraSendCmd::Execute() {
}

// ==========================================================================
// Make this return true when this Command no longer needs to run Execute.

bool BasicCameraSendCmd::IsFinished() {
	return false;
}

// ==========================================================================
// Called once after isFinished returns true

void BasicCameraSendCmd::End() {
}

// ==========================================================================
// Called when another command which requires this subsystem is scheduled to run

void BasicCameraSendCmd::Interrupted() {
	End();
}

// ==========================================================================
