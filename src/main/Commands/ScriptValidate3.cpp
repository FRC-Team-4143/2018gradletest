#include "Commands/ScriptValidate3.h"
#include <SmartDashboard/SmartDashboard.h>
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"
#include "Modules/ScriptCommandFactory3.h"

// ==========================================================================

ScriptValidate3::ScriptValidate3()
: ScriptValidate3("ScriptCommand3", "ScriptValid3") {
}

// ==========================================================================

ScriptValidate3::ScriptValidate3(std::string dashboardInput, std::string dashboardOutput)
: frc::Command("ScriptValidate3"), _dashboardInput(dashboardInput), _dashboardOutput(dashboardOutput) {
	std::string msg = GetName() + "::ctor(" + dashboardInput + ", " + dashboardOutput + ")";
	LOG(msg);

	SetRunWhenDisabled(true);
}

// ==========================================================================

void ScriptValidate3::Initialize() {
}

// ==========================================================================

void ScriptValidate3::Execute() {
	ValidateCommands();
}

// ==========================================================================

bool ScriptValidate3::IsFinished() {
	return true;
}

// ==========================================================================

void ScriptValidate3::End() {
}

// ==========================================================================

void ScriptValidate3::Interrupted() {
	End();
}

// ==========================================================================

void ScriptValidate3::ValidateCommands() {
	CommandListParser& parser(CommandListParser::GetInstance());
	auto script = SmartDashboard::GetString(_dashboardInput, "S(0)");
	auto valid = parser.IsValid(script);
	SmartDashboard::PutString(_dashboardOutput, valid ? "Valid" : "Invalid");
	ScriptCommandFactory3::GetInstance().SetBlueprint(valid ? script : "");
}

// ==========================================================================
