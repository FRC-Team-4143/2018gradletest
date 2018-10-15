#include "Commands/ArmOut.h"
#include "Robot.h"

ArmOut::ArmOut() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::arm);
}

void ArmOut::Initialize() {
}

void ArmOut::Execute() {
	Robot::arm->armOut(0.35); //0.5 for arm //1.0 for testing roller claw
}

bool ArmOut::IsFinished() {
	return false;
}

void ArmOut::End() {
	Robot::arm->armStop();
}

void ArmOut::Interrupted() {
	End();
}
