#pragma once

#include <Commands/CommandGroup.h>
#include <string>

// ==========================================================================

class ScriptCommand3 : public frc::CommandGroup {
public:
	ScriptCommand3(std::string name);
	ScriptCommand3(std::string name, std::string script);

	static void InitParameters();

protected:
	void ParseCommands(std::string commands);

private:
};

// ==========================================================================
