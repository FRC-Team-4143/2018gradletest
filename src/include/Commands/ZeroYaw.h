#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ZeroYaw : public frc::Command {
public:
	ZeroYaw();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
