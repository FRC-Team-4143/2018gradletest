#pragma once

#include <WPILib.h>

// ==========================================================================

class VisionSource : public frc::PIDSource {
public:
	VisionSource();
	virtual ~VisionSource();

	// PIDSource methods
	double PIDGet();
};

// ==========================================================================
