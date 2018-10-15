#pragma once

#include <Commands/Command.h>
#include "Subsystems/BasicCameraSub.h"

// ==========================================================================

class BasicCameraSendCmd : public frc::Command {
public:
	BasicCameraSendCmd(std::shared_ptr<BasicCameraSub> camera);

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
	std::shared_ptr<BasicCameraSub> m_camera;
};

// ==========================================================================
