#pragma once

#include <WPILib.h>
#include <condition_variable>

// ==========================================================================

class VisionSink : public frc::PIDOutput {
public:
	VisionSink();
	virtual ~VisionSink();

	// PIDOutput methods
	void PIDWrite(double output);

	double GetValue();

private:
	std::recursive_mutex _mutex;
	double _value;
};

// ==========================================================================
