#pragma once

#include <Commands/Command.h>

// ==========================================================================

class UnwindWheels : public frc::Command {
public:
	UnwindWheels();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
