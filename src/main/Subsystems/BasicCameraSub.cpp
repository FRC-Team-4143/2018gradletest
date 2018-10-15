#include "Subsystems/BasicCameraSub.h"
#include <CameraServer.h>
#include "Commands/Subsystem.h"

// ==========================================================================

BasicCameraSub::BasicCameraSub()
: m_enabled(false) {
}

// ==========================================================================

BasicCameraSub::~BasicCameraSub() {
}

// ==========================================================================

void BasicCameraSub::Enable() {
	if (!IsEnabled()) {
		_InitializeCamera();
		m_enabled = true;
	}
}

// ==========================================================================

bool BasicCameraSub::IsEnabled() const {
	return m_enabled;
}

// ==========================================================================

void BasicCameraSub::_InitializeCamera() {
	CameraServer::GetInstance()->StartAutomaticCapture();
}

// ==========================================================================
