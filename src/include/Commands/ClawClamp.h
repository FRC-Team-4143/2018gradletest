#pragma once

#include "Commands/Command.h"

class ClawClamp : public frc::Command {
public:
	ClawClamp();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
