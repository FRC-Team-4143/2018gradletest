#include <WPILib.h>
#include <string>
#include "Commands/ScriptCommand2.h"
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"

// ==========================================================================

ScriptCommand2::ScriptCommand2(std::string name)
: ScriptCommand2(name, SmartDashboard::GetString("ScriptCommand2", "")) {
}

// ==========================================================================

ScriptCommand2::ScriptCommand2(std::string name, std::string script)
: frc::CommandGroup(name) {
	LOG(GetName() + "::ctor");
	ParseCommands(script);
}

// ==========================================================================

void ScriptCommand2::InitParameters() {
	try {
		SmartDashboard::GetString("ScriptCommand2", "");
	} catch (...) {
		// SmartDashboard parameter does not exist; create it.
		SmartDashboard::PutString("ScriptCommand2", "S(1)");
	}
}

// ==========================================================================

void ScriptCommand2::ParseCommands(std::string script) {
	CommandListParser &parser(CommandListParser::GetInstance());
	parser.Parse(script, [this](bool parallel, Command *cmd, float timeout) {
		if (parallel) {
			if (timeout) {
				AddParallel(cmd, timeout);
			} else {
				AddParallel(cmd);
			}
		} else {
			if (timeout) {
				AddSequential(cmd, timeout);
			} else {
				AddSequential(cmd);
			}
		}
	});
}

// ==========================================================================
