#pragma once

#include <Commands/Command.h>
#include <string>

// ==========================================================================

class ScriptSleep : public frc::Command {
public:
	ScriptSleep(std::string name, float seconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	float _seconds;
};

// ==========================================================================
