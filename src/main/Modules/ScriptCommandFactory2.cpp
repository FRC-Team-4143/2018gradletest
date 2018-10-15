// ==========================================================================
// ScriptCommandFactory class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2017-01-28 JKSalmon - Initial development
// ==========================================================================
#include "Modules/ScriptCommandFactory2.h"
#include "Commands/ScriptCommand2.h"

// ==========================================================================

ScriptCommandFactory2::ScriptCommandFactory2()
: m_script(), m_pCommand() {
}

// ==========================================================================

ScriptCommandFactory2::~ScriptCommandFactory2() {
}

// ==========================================================================

ScriptCommandFactory2& ScriptCommandFactory2::GetInstance() {
	static ScriptCommandFactory2 instance;
	return instance;
}

// ==========================================================================

void ScriptCommandFactory2::SetBlueprint(std::string script, bool prepare) {
	if (m_script != script) {
		m_script = script;
		m_pCommand.reset();
	}

	if (prepare) {
		PrepareCommand();
	}
}

// ==========================================================================

std::unique_ptr<frc::Command> ScriptCommandFactory2::GetCommand() {
	PrepareCommand();
	return std::move(m_pCommand);
}

// ==========================================================================

void ScriptCommandFactory2::PrepareCommand() {
	if (!_HaveOne()) {
		_BuildOne();
	}
}

// ==========================================================================

void ScriptCommandFactory2::_BuildOne() {
	m_pCommand.reset(new ScriptCommand2("ScriptCommand2", m_script));
}

// ==========================================================================

bool ScriptCommandFactory2::_HaveOne() const {
	return !!m_pCommand;
}

// ==========================================================================
