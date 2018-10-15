#pragma once

#include <Commands/Command.h>

// ==========================================================================

class SetWheelOffsets : public frc::Command {
public:
	SetWheelOffsets();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
