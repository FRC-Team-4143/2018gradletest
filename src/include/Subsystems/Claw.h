#pragma once

#include <Commands/Subsystem.h>
#include "ctre/phoenix.h"

class Claw : public Subsystem {
	TalonSRX *clawMotor;


private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Claw();
	void InitDefaultCommand();
	void clawClamp(float speed);
	void clawUnclamp();
	void clawStop();
	float getClawPosition();
};
