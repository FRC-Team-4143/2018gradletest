#pragma once

#include <WPILib.h>
#include <Commands/Command.h>
#include <LiveWindow/LiveWindow.h>

#include "Commands/AutonomousCommand.h"
#include "OI.h"
#include "RobotMap.h"
#include "Subsystems/BasicCameraSub.h"
#include "Subsystems/DriveTrain.h"
#include "Subsystems/GyroSub.h"
#include "Subsystems/VisionBridgeSub.h"
#include "Subsystems/Climber.h"
#include "Subsystems/Elevator.h"
#include "Subsystems/Arm.h"
#include "Subsystems/Claw.h"
#include "Subsystems/Roller.h"

class Robot : public IterativeRobot {
private:
	frc::SendableChooser<int> chooser;
	frc::SendableChooser<int> chooser2;


public:
	Command *autonomousCommand;
	LiveWindow *lw = LiveWindow::GetInstance();
	DriverStation& ds = DriverStation::GetInstance();

	//static int switchSide;
	//static int scaleSide;
	static int inverter;
	static int climberArrangement;

	static OI *oi;
	static DriveTrain *driveTrain;
	static GyroSub *gyroSub;
	static std::shared_ptr<BasicCameraSub> basicCameraSub;
	static VisionBridgeSub *visionBridge;
	static Climber *climber;
	static Elevator *elevator;
	static Arm *arm;
	static Claw *claw;
	static Roller *roller;


	static Servo *servo;
	static Servo *servo2;

	virtual void RobotInit();
	virtual void RobotPeriodic();
	virtual void DisabledInit();
	virtual void DisabledPeriodic();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestPeriodic();

	void ScriptInit();
};
