#pragma once

#include <Commands/Subsystem.h>
#include "ctre/phoenix.h"

class Arm : public Subsystem {
	TalonSRX *armMotor;

private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Arm();
	void InitDefaultCommand();
	void armOut(float speed);
	void armIn(float speed);
	void armStop();
	float getArmPosition() const;
	bool IsInClimberRange() const;
};
