#include <WPILib.h>
#include <string>
#include "Commands/ScriptCommand3.h"
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"

// ==========================================================================

ScriptCommand3::ScriptCommand3(std::string name)
: ScriptCommand3(name, SmartDashboard::GetString("ScriptCommand3", "")) {
}

// ==========================================================================

ScriptCommand3::ScriptCommand3(std::string name, std::string script)
: frc::CommandGroup(name) {
	LOG(GetName() + "::ctor");
	ParseCommands(script);
}

// ==========================================================================

void ScriptCommand3::InitParameters() {
	try {
		SmartDashboard::GetString("ScriptCommand3", "");
	} catch (...) {
		// SmartDashboard parameter does not exist; create it.
		SmartDashboard::PutString("ScriptCommand3", "S(1)");
	}
}

// ==========================================================================

void ScriptCommand3::ParseCommands(std::string script) {
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
