#pragma once

#include <Commands/Command.h>

// ==========================================================================

class DriveDistance : public frc::Command {
public:
	DriveDistance (float twistAngle, float distance, float timeout);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	float _twistAngle;
	float _distance;
	float _timeout;
	int _count;
	int _fCount;
};

// ==========================================================================
