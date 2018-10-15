#include <WPILib.h>
#include <string>
#include "Commands/ScriptCommand.h"
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"

// ==========================================================================

ScriptCommand::ScriptCommand(std::string name)
: ScriptCommand(name, SmartDashboard::GetString("ScriptCommand", "")) {
}

// ==========================================================================

ScriptCommand::ScriptCommand(std::string name, std::string script)
: frc::CommandGroup(name) {
	LOG(GetName() + "::ctor");
	ParseCommands(script);
}

// ==========================================================================

void ScriptCommand::InitParameters() {
	try {
		SmartDashboard::GetString("ScriptCommand", "");
	} catch (...) {
		// SmartDashboard parameter does not exist; create it.
		SmartDashboard::PutString("ScriptCommand", "S(1)");
	}
}

// ==========================================================================

void ScriptCommand::ParseCommands(std::string script) {
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
