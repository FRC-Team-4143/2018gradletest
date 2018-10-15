#pragma once

#include <Commands/Command.h>

// ==========================================================================
// This command keeps the robot square to the edge of the field to which it
// is most square when the command is started.
// ==========================================================================

class GyroCrab : public frc::Command {
public:
	GyroCrab();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
	float _angle;
	bool flag;
};

// ==========================================================================
