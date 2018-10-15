#pragma once

#include "Commands/Command.h"

class ArmOut : public frc::Command {
public:
	ArmOut();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
