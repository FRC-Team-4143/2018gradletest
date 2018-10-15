#pragma once

#include "Commands/Command.h"

class Triggers : public frc::Command {
public:
	Triggers();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
