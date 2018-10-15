#pragma once

#include "Commands/Command.h"

class RollerOut : public frc::Command {
public:
	RollerOut();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
