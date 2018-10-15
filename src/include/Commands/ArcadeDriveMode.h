#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ArcadeDriveMode : public frc::Command {
public:
	ArcadeDriveMode();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
