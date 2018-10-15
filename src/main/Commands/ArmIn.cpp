#include "Commands/ArmIn.h"
#include "Robot.h"

ArmIn::ArmIn() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::arm);
}

void ArmIn::Initialize() {
}

void ArmIn::Execute() {
	Robot::arm->armIn(0.5); //0.5 for arm //1.0 for testing roller claw
}

bool ArmIn::IsFinished() {
	return false;
}

void ArmIn::End() {
	Robot::arm->armStop();
}

void ArmIn::Interrupted() {
	End();
}
