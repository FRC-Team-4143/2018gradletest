#pragma once

#include "Commands/Command.h"

class SetWheelsStraight : public frc::Command {
public:
	SetWheelsStraight();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
