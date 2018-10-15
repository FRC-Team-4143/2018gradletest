#pragma once

#include <Commands/Command.h>

class ScriptElevatorMovement : public frc::Command {
public:
	ScriptElevatorMovement(float power, float height, float timeout);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	float _power;
	float _height;
	float _timeout;
};
