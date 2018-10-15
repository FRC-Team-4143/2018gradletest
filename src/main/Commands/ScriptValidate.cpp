#include "Commands/ScriptValidate.h"
#include <SmartDashboard/SmartDashboard.h>
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"
#include "Modules/ScriptCommandFactory.h"

// ==========================================================================

ScriptValidate::ScriptValidate()
: ScriptValidate("ScriptCommand", "ScriptValid") {
}

// ==========================================================================

ScriptValidate::ScriptValidate(std::string dashboardInput, std::string dashboardOutput)
: frc::Command("ScriptValidate"), _dashboardInput(dashboardInput), _dashboardOutput(dashboardOutput) {
	std::string msg = GetName() + "::ctor(" + dashboardInput + ", " + dashboardOutput + ")";
	LOG(msg);

	SetRunWhenDisabled(true);
}

// ==========================================================================

void ScriptValidate::Initialize() {
}

// ==========================================================================

void ScriptValidate::Execute() {
	ValidateCommands();
}

// ==========================================================================

bool ScriptValidate::IsFinished() {
	return true;
}

// ==========================================================================

void ScriptValidate::End() {
}

// ==========================================================================

void ScriptValidate::Interrupted() {
	End();
}

// ==========================================================================

void ScriptValidate::ValidateCommands() {
	CommandListParser& parser(CommandListParser::GetInstance());
	auto script = SmartDashboard::GetString(_dashboardInput, "S(0)");
	auto valid = parser.IsValid(script);
	SmartDashboard::PutString(_dashboardOutput, valid ? "Valid" : "Invalid");
	ScriptCommandFactory::GetInstance().SetBlueprint(valid ? script : "");
}

// ==========================================================================
