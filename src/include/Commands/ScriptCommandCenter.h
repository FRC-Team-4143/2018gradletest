#pragma once

#include <Commands/CommandGroup.h>
#include <string>

// ==========================================================================

class ScriptCommandCenter : public frc::CommandGroup {
public:
	ScriptCommandCenter(std::string name);
	ScriptCommandCenter(std::string name, std::string script);

	static void InitParameters();

protected:
	void ParseCommands(std::string commands);

private:
};

// ==========================================================================
