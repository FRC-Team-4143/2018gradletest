#include "Modules/VisionSource.h"
#include "Robot.h"

// ==========================================================================

VisionSource::VisionSource() {
}

// ==========================================================================

VisionSource::~VisionSource() {
}

// ==========================================================================

double VisionSource::PIDGet() {
	return Robot::visionBridge->GetGearPosition();
}

// ==========================================================================
