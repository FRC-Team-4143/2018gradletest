#pragma once

#include "Commands/Command.h"

class ArmIn : public frc::Command {
public:
	ArmIn();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
