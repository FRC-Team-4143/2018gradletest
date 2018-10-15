#pragma once

#include "Commands/Command.h"

class ElevatorDown : public frc::Command {
public:
	ElevatorDown();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
