#pragma once

#include "Commands/Command.h"

class RollerIn : public frc::Command {
public:
	int count;

	RollerIn();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
