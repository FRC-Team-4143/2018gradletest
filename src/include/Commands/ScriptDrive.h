#pragma once

#include <Commands/Command.h>
#include <string>

// ==========================================================================

class ScriptDrive : public frc::Command {
public:
	ScriptDrive(std::string name, float x, float y, float z, float seconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	float _x;
	float _y;
	float _z;
	float _seconds;

	void _Cleanup();
};

// ==========================================================================
