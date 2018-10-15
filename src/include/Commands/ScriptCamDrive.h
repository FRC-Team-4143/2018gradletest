#pragma once

#include <Commands/Command.h>
#include <string>
#include "Modules/VisionSink.h"
#include "Modules/VisionSource.h"

// ==========================================================================

class ScriptCamDrive : public frc::Command {
public:
	ScriptCamDrive(std::string name, double x, double y, double maxSpeed, double seconds, int side);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	VisionSink _visionSink;
	VisionSource _visionSource;
	frc::PIDController* _pid;
	double _x;
	double _y;
	double _maxSpeed;
	double _timeoutSeconds;
	int _side;
	int _counter;
	double _angle;
	double _p;
	double _i;
	double _d;
	double _tol;
	double _center;
	bool _returnQuick;

	void _Cleanup();
};

// ==========================================================================
