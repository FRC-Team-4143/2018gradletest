#include "Commands/ScriptValidate2.h"
#include <SmartDashboard/SmartDashboard.h>
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"
#include "Modules/ScriptCommandFactory2.h"

// ==========================================================================

ScriptValidate2::ScriptValidate2()
: ScriptValidate2("ScriptCommand2", "ScriptValid2") {
}

// ==========================================================================

ScriptValidate2::ScriptValidate2(std::string dashboardInput, std::string dashboardOutput)
: frc::Command("ScriptValidate2"), _dashboardInput(dashboardInput), _dashboardOutput(dashboardOutput) {
	std::string msg = GetName() + "::ctor(" + dashboardInput + ", " + dashboardOutput + ")";
	LOG(msg);

	SetRunWhenDisabled(true);
}

// ==========================================================================

void ScriptValidate2::Initialize() {
}

// ==========================================================================

void ScriptValidate2::Execute() {
	ValidateCommands();
}

// ==========================================================================

bool ScriptValidate2::IsFinished() {
	return true;
}

// ==========================================================================

void ScriptValidate2::End() {
}

// ==========================================================================

void ScriptValidate2::Interrupted() {
	End();
}

// ==========================================================================

void ScriptValidate2::ValidateCommands() {
	CommandListParser& parser(CommandListParser::GetInstance());
	auto script = SmartDashboard::GetString(_dashboardInput, "S(0)");
	auto valid = parser.IsValid(script);
	SmartDashboard::PutString(_dashboardOutput, valid ? "Valid" : "Invalid");
	ScriptCommandFactory2::GetInstance().SetBlueprint(valid ? script : "");
}

// ==========================================================================
