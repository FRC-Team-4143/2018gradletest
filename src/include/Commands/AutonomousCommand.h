#pragma once

#include <Commands/Command.h>

// ==========================================================================

class AutonomousCommand : public frc::Command {
public:
	AutonomousCommand();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

// ==========================================================================
