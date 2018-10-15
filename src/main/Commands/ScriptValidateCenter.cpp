#include "Commands/ScriptValidateCenter.h"
#include <SmartDashboard/SmartDashboard.h>
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"
#include "Modules/ScriptCommandFactoryCenter.h"

// ==========================================================================

ScriptValidateCenter::ScriptValidateCenter()
: ScriptValidateCenter("ScriptCommandCenter", "ScriptValidCenter") {
}

// ==========================================================================

ScriptValidateCenter::ScriptValidateCenter(std::string dashboardInput, std::string dashboardOutput)
: frc::Command("ScriptValidateCenter"), _dashboardInput(dashboardInput), _dashboardOutput(dashboardOutput) {
	std::string msg = GetName() + "::ctor(" + dashboardInput + ", " + dashboardOutput + ")";
	LOG(msg);

	SetRunWhenDisabled(true);
}

// ==========================================================================

void ScriptValidateCenter::Initialize() {
}

// ==========================================================================

void ScriptValidateCenter::Execute() {
	ValidateCommands();
}

// ==========================================================================

bool ScriptValidateCenter::IsFinished() {
	return true;
}

// ==========================================================================

void ScriptValidateCenter::End() {
}

// ==========================================================================

void ScriptValidateCenter::Interrupted() {
	End();
}

// ==========================================================================

void ScriptValidateCenter::ValidateCommands() {
	CommandListParser& parser(CommandListParser::GetInstance());
	auto script = SmartDashboard::GetString(_dashboardInput, "S(0)");
	auto valid = parser.IsValid(script);
	SmartDashboard::PutString(_dashboardOutput, valid ? "Valid" : "Invalid");
	ScriptCommandFactoryCenter::GetInstance().SetBlueprint(valid ? script : "");
}

// ==========================================================================
