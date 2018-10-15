#pragma once

#include <Commands/Subsystem.h>
#include "ctre/phoenix.h"

class Elevator : public Subsystem {
	TalonSRX *elevatorMotor;

private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Elevator();
	void InitDefaultCommand();
	void elevatorDown(float speed);
	void elevatorUp(float speed);
	void elevatorStop();
	float getElevatorPosition() const;
	bool IsInClimberRange() const;
};
