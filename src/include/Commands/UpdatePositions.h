#pragma once

#include <Commands/Command.h>

// ==========================================================================

class UpdatePositions : public frc::Command {
public:
	UpdatePositions();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
