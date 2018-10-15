#include "OI.h"
#include "RobotMap.h"
#include <SmartDashboard/SmartDashboard.h>
#include "Commands/ArcadeDriveMode.h"
#include "Commands/ArmIn.h"
#include "Commands/ArmOut.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/BasicCameraEnableCmd.h"
#include "Commands/ClawClamp.h"
#include "Commands/ClawUnclamp.h"
#include "Commands/Climb.h"
#include "Commands/CrabDrive.h"
#include "Commands/ElevatorDown.h"
#include "Commands/ElevatorUp.h"
#include "Commands/FieldCentric.h"
#include "Commands/GyroCrab.h"
#include "Commands/GyroStraighten.h"
#include "Commands/ReverseClimb.h"
#include "Commands/RollerIn.h"
#include "Commands/RollerOut.h"
#include "Commands/ScriptCamDrive.h"
#include "Commands/ScriptValidate.h"
#include "Commands/ScriptValidate2.h"
#include "Commands/ScriptValidate3.h"
#include "Commands/ScriptValidateCenter.h"
#include "Commands/SetWheelOffsets.h"
#include "Commands/SetWheelsStraight.h"
#include "Commands/UnwindWheels.h"
#include "Commands/UpdatePositions.h"
#include "Commands/ZeroYaw.h"
#include "Commands/ClawStop.h"
#include "Robot.h"
#include <cmath>

const uint32_t JOYSTICK_LX_AXIS = 0;
const uint32_t JOYSTICK_LY_AXIS = 1;
const uint32_t JOYSTICK_LTRIG_AXIS = 2;
const uint32_t JOYSTICK_RTRIG_AXIS = 3;
const uint32_t JOYSTICK_RX_AXIS = 4;
const uint32_t JOYSTICK_RY_AXIS = 5;

const uint32_t JOYSTICK_BUTTON_A = 1;
const uint32_t JOYSTICK_BUTTON_B = 2;
const uint32_t JOYSTICK_BUTTON_X = 3;
const uint32_t JOYSTICK_BUTTON_Y = 4;
const uint32_t JOYSTICK_BUTTON_LB = 5;
const uint32_t JOYSTICK_BUTTON_RB = 6;
const uint32_t JOYSTICK_BUTTON_BACK = 7;
const uint32_t JOYSTICK_BUTTON_START = 8;
const uint32_t JOYSTICK_BUTTON_LEFT = 9;
const uint32_t JOYSTICK_BUTTON_RIGHT = 10;
//const float JOYSTICK_DEAD_ZONE;

const uint32_t JOYSTICK_BUTTON_TRIG = 1;
const uint32_t JOYSTICK_BUTTON_2 = 2;
const uint32_t JOYSTICK_BUTTON_3 = 3;
const uint32_t JOYSTICK_BUTTON_4 = 4;
const uint32_t JOYSTICK_BUTTON_5 = 5;
const uint32_t JOYSTICK_BUTTON_6 = 6;
const uint32_t JOYSTICK_BUTTON_7 = 7;
const uint32_t JOYSTICK_BUTTON_10 = 10;

// ==========================================================================

OI::OI() {
	driverJoystick = new Joystick(0);
	//driverJoystick2 = new Joystick(1);

	armIn = new ArmIn();
	armOut = new ArmOut();
	clawClamp = new ClawClamp();
	clawUnclamp = new ClawUnclamp();
	climb = new Climb();
	crabDrive = new CrabDrive();
	elevatorDown = new ElevatorDown();
	elevatorUp = new ElevatorUp();
	fieldCentric = new FieldCentric();
	gyroCrab = new GyroCrab();
	gyroStraighten = new GyroStraighten();
	rollerIn = new RollerIn();
	rollerOut = new RollerOut();
	setWheelsStraight = new SetWheelsStraight();
	unwindWheels = new UnwindWheels();
	zeroYaw = new ZeroYaw();
	stopClaw = new ClawStop();


	//arcade = new ArcadeDriveMode();

	auto cameraEnableCmd = new BasicCameraEnableCmd(Robot::basicCameraSub);

////Main Driver Controller///----------------------------------------------------------------

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_BACK))->WhileHeld(stopClaw);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_START))->WhileHeld(climb);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_LEFT))->ToggleWhenPressed(crabDrive);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_RIGHT))->WhileHeld(zeroYaw);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_LB))->WhenPressed(clawClamp);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_RB))->WhenPressed(clawUnclamp);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_Y))->WhileHeld(armOut);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_A))->WhileHeld(armIn);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_B))->WhileHeld(rollerOut);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_RB))->WhileHeld(rollerIn);


////-----------------------------------------------------------------------------------------

	//SmartDashboard::PutData("Camera On", cameraEnableCmd);

	//SmartDashboard::PutData("Camera", useCamera);
	SmartDashboard::PutData("SetWheelOffsets", new SetWheelOffsets());
	//SmartDashboard::PutData("Zero Yaw", new ZeroYaw());

	SmartDashboard::PutData("Update Positions", new UpdatePositions());
	SmartDashboard::PutData("Validate Script", new ScriptValidate());
	SmartDashboard::PutData("Validate Script 2", new ScriptValidate2());
	SmartDashboard::PutData("Validate Script 3", new ScriptValidate3());
	SmartDashboard::PutData("Validate Script Center", new ScriptValidateCenter());

	//SmartDashboard::PutData("Validate All Scripts", new ScriptValidateAll());

	//SmartDashboard::PutData("JAM SHOOTER", new TestJamShooter());
}

// ==========================================================================

bool OI::IsInClimbMode() const {
	return (GetDriverJoystick()->GetRawButton(JOYSTICK_BUTTON_X) == 1);
}

// ==========================================================================

bool OI::GetRightBumper() {
	auto value = driverJoystick->GetRawButton(JOYSTICK_BUTTON_RB);
	return value;
}

// ==========================================================================

bool OI::GetLeftBumper() {
	auto value = driverJoystick->GetRawButton(JOYSTICK_BUTTON_LB);
	return value;
}

// ==========================================================================

float OI::GetRightTrigger() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_RTRIG_AXIS);
	return (fabs(value) <= JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetLeftTrigger() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_LTRIG_AXIS);
	return (fabs(value) <= JOYSTICK_DEAD_ZONE) ? 0 : fabs(value);
}

// ==========================================================================

float OI::GetJoystickX() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_LX_AXIS);
	return (fabs(value) <= JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystickRX() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_RX_AXIS);
	return (fabs(value) <= JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystickY() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_LY_AXIS);
	return (fabs(value) <= JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystickZ() {
	auto value = GetDriverJoystick()->GetRawAxis(JOYSTICK_RX_AXIS);
	return (fabs(value) <= JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

bool OI::GetButtonX() {
	auto value = GetDriverJoystick()->GetRawButton(JOYSTICK_BUTTON_X);
	return value;
}

// ==========================================================================

bool OI::_GetButtonBack() const {
	auto value = GetDriverJoystick()->GetRawButton(JOYSTICK_BUTTON_BACK);
	return value;
}

// ==========================================================================
