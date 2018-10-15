#pragma once

#include "Commands/Command.h"

class ClawStop : public frc::Command {
public:
	ClawStop();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
