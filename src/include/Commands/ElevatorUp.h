#pragma once

#include "Commands/Command.h"

class ElevatorUp : public frc::Command {
public:

	ElevatorUp();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
