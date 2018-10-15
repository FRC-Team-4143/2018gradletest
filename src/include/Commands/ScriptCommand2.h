#pragma once

#include <Commands/CommandGroup.h>
#include <string>

// ==========================================================================

class ScriptCommand2 : public frc::CommandGroup {
public:
	ScriptCommand2(std::string name);
	ScriptCommand2(std::string name, std::string script);

	static void InitParameters();

protected:
	void ParseCommands(std::string commands);

private:
};

// ==========================================================================
