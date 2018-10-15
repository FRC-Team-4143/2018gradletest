#pragma once

#include <Commands/Command.h>

// ==========================================================================

class DriveTilVision : public frc::Command {
public:
	DriveTilVision(double y, double x, double twist, double seconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double _x;
	double _y;
	double _twist;
	double _seconds;
};

// ==========================================================================
