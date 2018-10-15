#pragma once

#include <WPILib.h>

// ==========================================================================

class OI {
private:


public:
	OI();
	Joystick* driverJoystick;
	//Joystick* driverJoystick2;

	const float JOYSTICK_DEAD_ZONE = 0.15;

	bool IsInClimbMode() const;

	bool _GetButtonBack() const;

	float GetRightTrigger();
	float GetLeftTrigger();
	bool GetRightBumper();
	bool GetLeftBumper();
	float GetJoystickRX();
	float GetJoystickX();
	float GetJoystickY();
	float GetJoystickZ();
	bool GetButtonX();
	bool GetButtonTrig();
	bool GetButton2();
	bool GetButton6();
	bool GetButton10();
	bool GetButton5();

	Command* zeroYaw;
	Command* unwindWheels;
	//Command* arcade;
	Command* gyroCrab;
	Command* fieldCentric;
	Command* climb;
	Command* elevatorDown;
	Command* elevatorUp;
	Command* armIn;
	Command* armOut;
	Command* clawClamp;
	Command* clawUnclamp;
	Command* setWheelsStraight;
	Command* gyroStraighten;
	Command* crabDrive;
	Command* rollerOut;
	Command* rollerIn;
	Command* stopClaw;

	Joystick* GetDriverJoystick() const { return driverJoystick; }
};

// ==========================================================================
