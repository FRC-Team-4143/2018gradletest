#include <WPILib.h>
#include <string>
#include "Commands/ScriptCommandCenter.h"
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"

// ==========================================================================

ScriptCommandCenter::ScriptCommandCenter(std::string name)
: ScriptCommandCenter(name, SmartDashboard::GetString("ScriptCommandCenter", "")) {
}

// ==========================================================================

ScriptCommandCenter::ScriptCommandCenter(std::string name, std::string script)
: frc::CommandGroup(name) {
	LOG(GetName() + "::ctor");
	ParseCommands(script);
}

// ==========================================================================

void ScriptCommandCenter::InitParameters() {
	try {
		SmartDashboard::GetString("ScriptCommandCenter", "");
	} catch (...) {
		// SmartDashboard parameter does not exist; create it.
		SmartDashboard::PutString("ScriptCommandCenter", "S(1)");
	}
}

// ==========================================================================

void ScriptCommandCenter::ParseCommands(std::string script) {
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
