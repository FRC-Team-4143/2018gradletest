#pragma once

#include "Commands/Command.h"

class Climb : public frc::Command {
public:
	Climb();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
