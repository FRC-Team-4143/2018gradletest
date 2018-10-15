#pragma once

#include <Commands/Command.h>

class ScriptArmMovement : public frc::Command {
public:
	ScriptArmMovement(float power, float distance, float timeout);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	float _power;
	float _distance;
	float _timeout;
};
