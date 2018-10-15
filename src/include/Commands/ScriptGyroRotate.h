#pragma once

#include <Commands/Command.h>
#include <string>

// ==========================================================================

class ScriptGyroRotate : public frc::Command {
public:
	ScriptGyroRotate(std::string name, float desiredangle, double power, float seconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	float _desiredangle;
	double _power;
	float _seconds;
	int _counter;

	void _Cleanup();
};

// ==========================================================================
