#pragma once

#include <Commands/Command.h>

class ScriptClawOpen : public frc::Command {
public:
	ScriptClawOpen(float timeout);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	float _timeout;
};
