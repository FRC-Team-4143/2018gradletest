#pragma once

#include <Commands/Command.h>

class ScriptClawClose : public frc::Command {
public:
	ScriptClawClose(float timeout);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	float _timeout;
};
