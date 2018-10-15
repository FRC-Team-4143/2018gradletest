	#pragma once

#include <Commands/Command.h>

// ==========================================================================

class CrabDrive : public frc::Command {
public:
	CrabDrive();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

// ==========================================================================
