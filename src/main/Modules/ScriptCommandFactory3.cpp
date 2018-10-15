// ==========================================================================
// ScriptCommandFactory class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2017-01-28 JKSalmon - Initial development
// ==========================================================================
#include "Modules/ScriptCommandFactory3.h"
#include "Commands/ScriptCommand3.h"

// ==========================================================================

ScriptCommandFactory3::ScriptCommandFactory3()
: m_script(), m_pCommand() {
}

// ==========================================================================

ScriptCommandFactory3::~ScriptCommandFactory3() {
}

// ==========================================================================

ScriptCommandFactory3& ScriptCommandFactory3::GetInstance() {
	static ScriptCommandFactory3 instance;
	return instance;
}

// ==========================================================================

void ScriptCommandFactory3::SetBlueprint(std::string script, bool prepare) {
	if (m_script != script) {
		m_script = script;
		m_pCommand.reset();
	}

	if (prepare) {
		PrepareCommand();
	}
}

// ==========================================================================

std::unique_ptr<frc::Command> ScriptCommandFactory3::GetCommand() {
	PrepareCommand();
	return std::move(m_pCommand);
}

// ==========================================================================

void ScriptCommandFactory3::PrepareCommand() {
	if (!_HaveOne()) {
		_BuildOne();
	}
}

// ==========================================================================

void ScriptCommandFactory3::_BuildOne() {
	m_pCommand.reset(new ScriptCommand3("ScriptCommand3", m_script));
}

// ==========================================================================

bool ScriptCommandFactory3::_HaveOne() const {
	return !!m_pCommand;
}

// ==========================================================================
