// ==========================================================================
// ScriptCommandFactory class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2017-01-28 JKSalmon - Initial development
// ==========================================================================
#include "Modules/ScriptCommandFactoryCenter.h"
#include "Commands/ScriptCommandCenter.h"

// ==========================================================================

ScriptCommandFactoryCenter::ScriptCommandFactoryCenter()
: m_script(), m_pCommand() {
}

// ==========================================================================

ScriptCommandFactoryCenter::~ScriptCommandFactoryCenter() {
}

// ==========================================================================

ScriptCommandFactoryCenter& ScriptCommandFactoryCenter::GetInstance() {
	static ScriptCommandFactoryCenter instance;
	return instance;
}

// ==========================================================================

void ScriptCommandFactoryCenter::SetBlueprint(std::string script, bool prepare) {
	if (m_script != script) {
		m_script = script;
		m_pCommand.reset();
	}

	if (prepare) {
		PrepareCommand();
	}
}

// ==========================================================================

std::unique_ptr<frc::Command> ScriptCommandFactoryCenter::GetCommand() {
	PrepareCommand();
	return std::move(m_pCommand);
}

// ==========================================================================

void ScriptCommandFactoryCenter::PrepareCommand() {
	if (!_HaveOne()) {
		_BuildOne();
	}
}

// ==========================================================================

void ScriptCommandFactoryCenter::_BuildOne() {
	m_pCommand.reset(new ScriptCommandCenter("ScriptCommandCenter", m_script));
}

// ==========================================================================

bool ScriptCommandFactoryCenter::_HaveOne() const {
	return !!m_pCommand;
}

// ==========================================================================
