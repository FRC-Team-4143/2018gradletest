#pragma once

#include "Commands/Command.h"

class ClawUnclamp : public frc::Command {
public:
	ClawUnclamp();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
