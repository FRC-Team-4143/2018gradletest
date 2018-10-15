#pragma once

#include <Commands/Command.h>
#include <string>

// ==========================================================================

class ScriptValidate2 : public frc::Command {
public:
	ScriptValidate2();
	ScriptValidate2(std::string dashboardInput, std::string dashboardOutput);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

protected:
	void ValidateCommands();

private:
	std::string _dashboardInput;
	std::string _dashboardOutput;
};

// ==========================================================================
