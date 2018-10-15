#pragma once

#include <Commands/Command.h>

// ==========================================================================

class SetWheelsTwist : public frc::Command {
public:
	SetWheelsTwist(double seconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double _seconds;
};

// ==========================================================================
