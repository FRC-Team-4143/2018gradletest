#pragma once

#include <Commands/Command.h>

class ScriptRollers : public frc::Command {
public:
	ScriptRollers(float power, float timeout);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	float _power;
	float _timeout;
};
