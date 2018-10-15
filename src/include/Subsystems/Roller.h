#pragma once

#include <Commands/Subsystem.h>
#include "ctre/phoenix.h"

class Roller : public Subsystem {
	TalonSRX *rollerMotor;

private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Roller();
	void InitDefaultCommand();
	void rollerOut(float speed);
	void rollerIn(float speed);
	void rollerStop();

};
