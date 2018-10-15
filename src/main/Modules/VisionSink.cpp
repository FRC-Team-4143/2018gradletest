#include "Modules/VisionSink.h"

// ==========================================================================

VisionSink::VisionSink()
: _mutex(), _value(0) {
}

// ==========================================================================

VisionSink::~VisionSink() {
}

// ==========================================================================

void VisionSink::PIDWrite(double output) {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	_value = output;
}

// ==========================================================================

double VisionSink::GetValue() {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	return _value;
}

// ==========================================================================
