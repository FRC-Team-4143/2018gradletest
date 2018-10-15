#include "Robot.h"
#include <iostream>
#include "Commands/DriveDistance.h"
#include "Commands/ScriptCamDrive.h"
#include "Commands/ScriptCommand.h"
#include "Commands/ScriptCommand2.h"
#include "Commands/ScriptCommand3.h"
#include "Commands/ScriptCommandCenter.h"
#include "Commands/ScriptDrive.h"
#include "Commands/ScriptFieldCentricCrab.h"
#include "Commands/ScriptGyroDrive.h"
#include "Commands/ScriptSleep.h"
#include "Commands/ScriptElevatorMovement.h"
#include "Commands/ScriptClawOpen.h"
#include "Commands/ScriptClawClose.h"
#include "Commands/ScriptArmMovement.h"
#include "Commands/WaitForVision.h"
#include "Commands/ZeroYaw.h"
#include "Commands/ScriptGyroRotate.h"
#include "Commands/ScriptRollers.h"
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"
#include "Modules/ScriptCommandFactory.h"
#include "Modules/ScriptCommandFactory2.h"
#include "Modules/ScriptCommandFactory3.h"
#include "Modules/ScriptCommandFactoryCenter.h"

OI* Robot::oi;
GyroSub* Robot::gyroSub = nullptr;
DriveTrain* Robot::driveTrain = nullptr;
std::shared_ptr<BasicCameraSub> Robot::basicCameraSub;
VisionBridgeSub* Robot::visionBridge = nullptr;
Climber* Robot::climber = nullptr;
Servo* Robot::servo = nullptr;
Servo* Robot::servo2 = nullptr;
Arm* Robot::arm = nullptr;
Elevator* Robot::elevator = nullptr;
Claw* Robot::claw = nullptr;
Roller* Robot::roller = nullptr;

int Robot::inverter = 1;
int Robot::climberArrangement = 1;

void Robot::RobotInit() {

	chooser.AddObject("Left", -1);
	chooser.AddDefault("Center", 0);
	chooser.AddObject("Right", 1);
	frc::SmartDashboard::PutData("Starting Position", &chooser);

	chooser2.AddDefault("Climber Deattached", 1);
	chooser2.AddObject("Climber Attached", 0);
	frc::SmartDashboard::PutData("Climber", &chooser2);

	Preferences::GetInstance();

	RobotMap::Initialize();

	ScriptInit();
	SmartDashboard::PutString("ScriptCommand", "P: CC(1) A(0.5, 4096, 1) S(1) DD(1, 25.5, 5) E(1, 6.5, 5) GR(0.3, -80, 2) R(-1, 1)");
	SmartDashboard::PutString("ScriptCommand2", "P: CC(1) A(0.5, 4096, 1) S(1) DD(0, 14, 5) E(1, 3, 3) GR(0.3, -80, 1.5) DD(0, 3, 3) CO(1)");
	SmartDashboard::PutString("ScriptCommand3", "P: CC(1) A(0.5, 4096, 1) S(1) DD(0, 18.5, 5) DD(270, 16, 10)  E(1, 6.5, 5) DD(0, 4, 5) R(-0.35, 1) CO(1)");
	SmartDashboard::PutString("ScriptCommandCenter", "P: CC(1) A(0.5, 4096, 1) S(1) E(1, 3, 3) DD(45, 7, 5) DD(0, 5, 5) R(-0.35, 1) CO(1) CC(1) CO(1)");
	SmartDashboard::PutString("ScriptValid", "");
	SmartDashboard::PutString("ScriptValid2", "");
	SmartDashboard::PutString("ScriptValid3", "");
	SmartDashboard::PutString("ScriptValidCenter", "");
	//SmartDashboard::PutNumber("Twist Angle", 0);

	gyroSub = new GyroSub();
	driveTrain = new DriveTrain();
	basicCameraSub.reset(new BasicCameraSub());
	visionBridge = new VisionBridgeSub();
	climber = new Climber();
	driveTrain->SetWheelbase(21.5, 24, 22.8);
	driveTrain->loadWheelOffsets();
	elevator = new Elevator();
	arm = new Arm();
	claw = new Claw();
	roller = new Roller();
	oi = new OI();

	servo = new Servo(0);
	servo2 = new Servo(1);
}

void Robot::RobotPeriodic() {
	driveTrain->Dashboard();

	climberArrangement = chooser2.GetSelected();
	//std::cout << "climberArrangement = " << climberArrangement << std::endl;

	SmartDashboard::PutNumber("Elevator Encoder Count", elevator->getElevatorPosition());
	SmartDashboard::PutNumber("Arm Encoder Count", arm->getArmPosition());
	SmartDashboard::PutNumber("Clamp Encoder Count", RobotMap::clawMotor->GetSelectedSensorPosition(0));

	SmartDashboard::PutNumber("Gyro Yaw", RobotMap::imu->GetYaw());
	//SmartDashboard::PutNumber("Joystick X", Robot::oi->driverJoystick->GetRawAxis(0));
	//SmartDashboard::PutNumber("Joystick Y", Robot::oi->driverJoystick->GetRawAxis(1));

	SmartDashboard::PutNumber("Inverter", Robot::inverter);

	//SmartDashboard::PutNumber("FLS Error", RobotMap::driveTrainFrontLeftSteer->GetClosedLoopError(0));
	//SmartDashboard::PutNumber("FL Velocity", RobotMap::driveTrainFrontLeftDrive->GetSelectedSensorVelocity(0));
	//SmartDashboard::PutNumber("RL Velocity", RobotMap::driveTrainRearLeftDrive->GetSelectedSensorVelocity(0));
	//SmartDashboard::PutNumber("Elevator Current", RobotMap::pdp->GetCurrent(8)); // 3 for Test Bot
	//SmartDashboard::PutNumber("Claw Current", RobotMap::pdp->GetCurrent(2));


}

void Robot::DisabledInit() {
	//char mode = 1;
	//RobotMap::serialPort->Write(&mode, 1);
	//RobotMap::serialPort1->Write(&mode, 1);
	//RobotMap::serialPort2->Write(&mode, 1);
	//SmartDashboard::PutNumber("Serial 0", 1);
	//SmartDashboard::PutNumber("Serial 1", 1);
	//SmartDashboard::PutNumber("Serial 2", 1);
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
	//driveTrain->Dashboard();
	SmartDashboard::PutNumber("Gyro Yaw", RobotMap::imu->GetYaw());
	//indexer->ReadPDP();

	//char mode = SmartDashboard::GetNumber("Serial 0", 1);
	//RobotMap::serialPort->Write(&mode, 1);
	//char mode1 = SmartDashboard::GetNumber("Serial 1", 1);
	//RobotMap::serialPort1->Write(&mode1, 1);
	//char mode2 = SmartDashboard::GetNumber("Serial 2", 1);
	//RobotMap::serialPort2->Write(&mode2, 1);

	driveTrain->readLidar();
}

void Robot::AutonomousInit() {
	printf("Match time start: %f\r\n", DriverStation::GetInstance().GetMatchTime());
	driveTrain->enableSteeringPID();
	RobotMap::imu->ZeroYaw();

	servo->SetAngle(55);

	RobotMap::driveTrainFrontLeftDrive->SetNeutralMode(NeutralMode::Coast);
	RobotMap::driveTrainFrontRightDrive->SetNeutralMode(NeutralMode::Coast);
	RobotMap::driveTrainRearLeftDrive->SetNeutralMode(NeutralMode::Coast);
	RobotMap::driveTrainRearRightDrive->SetNeutralMode(NeutralMode::Coast);

	RobotMap::elevatorMotor->SetSelectedSensorPosition(0, 0, 10); //sets elevator to 0 counts at start
	RobotMap::armMotor->SetSelectedSensorPosition(0, 0, 10); // sets arm to 0 at start
	//RobotMap::clawMotor->SetSelectedSensorPosition(0, 0, 10); //sets claw to 0 counts at start

	std::string gameData;
	bool scaleIsLeft = false;
	bool scaleIsRight = false;
	bool switchIsLeft = false;
	bool switchIsRight = false;

	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

	if (gameData[0] == 'L') {
		//Left Switch Ownership
		//switchSide = -1;
		switchIsLeft = true;
	}
	else {
		//Right Switch Ownership
		//switchSide = 1;
		switchIsRight = true;
	}

	if (gameData[1] == 'L') {
		//Left Scale Ownership
		//scaleSide = -1;
		scaleIsLeft = true;
	}
	else {
		//Right Scale Ownership
		//scaleSide = 1;
		scaleIsRight = true;
	}

	int startingPosition = chooser.GetSelected();
	std::cout << "Starting Position = " << startingPosition << std::endl;

	bool robotIsLeft = -1 == startingPosition;
	bool robotIsCenter = 0 == startingPosition;
	bool robotIsRight = 1 == startingPosition;

	inverter = 1;
	if (robotIsLeft || (robotIsCenter && switchIsLeft)) {
		inverter = -1;
	}




		//char mode = 3;
	//RobotMap::serialPort->Write(&mode, 1);
	//RobotMap::serialPort1->Write(&mode, 1);
	//RobotMap::serialPort2->Write(&mode, 1);
	//SmartDashboard::PutNumber("Serial 0", 3);
	//SmartDashboard::PutNumber("Serial 1", 3);
	//SmartDashboard::PutNumber("Serial 2", 3);

	printf("Before new ScriptCommand: %f\r\n", DriverStation::GetInstance().GetMatchTime());

	if (robotIsCenter) {
		printf("Selected Script Center.\r\n");
		autonomousCommand = ScriptCommandFactoryCenter::GetInstance().GetCommand().release();
	}
	else if ((robotIsLeft && scaleIsLeft) || (robotIsRight && scaleIsRight)) {
		printf("Selected Script 1 (Scale).\r\n");
		autonomousCommand = ScriptCommandFactory::GetInstance().GetCommand().release();
	}
	else if ((robotIsLeft && switchIsLeft) || (robotIsRight && switchIsRight)) {
		printf("Selected Script 2. (Switch)\r\n");
		autonomousCommand = ScriptCommandFactory2::GetInstance().GetCommand().release();
	}
	else {
		printf("Selected Script 3. (Else)\r\n");
		autonomousCommand = ScriptCommandFactory3::GetInstance().GetCommand().release();
	}

	printf("After new ScriptCommand: %f\r\n", DriverStation::GetInstance().GetMatchTime());

	if (autonomousCommand != nullptr) {
		autonomousCommand->Start();
	}

	printf("Match time end of init: %f\r\n", DriverStation::GetInstance().GetMatchTime());
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();

	SmartDashboard::PutNumber("Gyro Yaw", RobotMap::imu->GetYaw());

	driveTrain->readLidar();
}

void Robot::TeleopInit() {
	driveTrain->enableSteeringPID();
	driveTrain->disableSpeedControl();

	servo->SetAngle(55);


	RobotMap::driveTrainFrontLeftDrive->SetNeutralMode(NeutralMode::Coast);
	RobotMap::driveTrainFrontRightDrive->SetNeutralMode(NeutralMode::Coast);
	RobotMap::driveTrainRearLeftDrive->SetNeutralMode(NeutralMode::Coast);
	RobotMap::driveTrainRearRightDrive->SetNeutralMode(NeutralMode::Coast);
	RobotMap::climbingMotor->SetNeutralMode(NeutralMode::Brake);
	RobotMap::armMotor->SetNeutralMode(NeutralMode::Brake);

	//char mode = 3;
	//RobotMap::serialPort->Write(&mode, 1);
	//RobotMap::serialPort1->Write(&mode, 1);
	//RobotMap::serialPort2->Write(&mode, 1);
	//SmartDashboard::PutNumber("Serial 0", 3);
	//SmartDashboard::PutNumber("Serial 1", 3);
	//SmartDashboard::PutNumber("Serial 2", 3);

	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// these lines or comment it out.
	if (autonomousCommand != nullptr) {
		autonomousCommand->Cancel();
	}
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
	//indexer->ReadPDP();
	//driveTrain->Dashboard();

	//char mode = SmartDashboard::GetNumber("Serial 0", 3);
	//RobotMap::serialPort->Write(&mode, 1);
	//char mode1 = SmartDashboard::GetNumber("Serial 1", 3);
	//RobotMap::serialPort1->Write(&mode1, 1);
	//char mode2 = SmartDashboard::GetNumber("Serial 2", 3);
	//RobotMap::serialPort2->Write(&mode2, 1);

	driveTrain->readLidar();

	//make controller rumble at 30 seconds left
	/*if (ds.GetMatchTime() < 30 && ds.GetMatchTime() > 25) {
		Robot::oi->GetDriverJoystick()->SetRumble(Joystick::kLeftRumble, 1);
		Robot::oi->GetDriverJoystick()->SetRumble(Joystick::kRightRumble, 1);
	} else {
		Robot::oi->GetDriverJoystick()->SetRumble(Joystick::kLeftRumble, 0);
		Robot::oi->GetDriverJoystick()->SetRumble(Joystick::kRightRumble, 0);
	}*/

}

void Robot::TestPeriodic() {
}

void Robot::ScriptInit() {
	LOG("Robot::ScriptInit");

	CommandListParser &parser(CommandListParser::GetInstance());

	parser.AddCommand(CommandParseInfo(
			"Drive", {"D", "d"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto y = parameters[0];
		auto x = parameters[1];
		auto z = parameters[2];
		auto timeout = parameters[3];
		Command *command = new ScriptDrive("Drive", y, x, z, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"DriveDistace", {"DD", "dd"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(3);
		auto twistAngle = parameters[0];
		auto distance = parameters[1];
		auto timeout = parameters[2];
		 //if (0 == timeout) timeout = 4;
		Command *command = new DriveDistance(twistAngle, distance, timeout);

		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"WaitForVision", {"WAIT", "wait"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(0);
		Command *command = new WaitForVision();
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"Sleep", {"S", "s"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(1);
		auto timeout = parameters[0];
		Command *command = new ScriptSleep("Sleep", timeout);
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"DriveGyro", {"DG", "dg"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto y = parameters[0];
		auto x = parameters[1];
		auto desiredangle = parameters[2];
		auto timeout = parameters[3];
		Command *command = new ScriptGyroDrive("DriveGyro", y, x, desiredangle, timeout);
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"GyroRotate", {"GR", "gr"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(3);
		auto power = parameters[0];
		auto desiredAngle = parameters[1];
		auto timeout = parameters[2];
		Command *command = new ScriptGyroRotate("GyroRotate", desiredAngle, power, timeout);
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"FieldCentricDrive", {"FC", "fc"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto y = parameters[0];
		auto x = parameters[1];
		auto z = parameters[2];
		auto timeout = parameters[3];
		Command *command = new ScriptFieldCentricCrab(z, y, x, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
				"ElevatorMovement", {"E", "e"},
				[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
			parameters.resize(3);
			auto power = parameters[0];
			auto height = parameters[1];
			auto timeout = parameters[2];
						Command *command = new ScriptElevatorMovement(power, height, timeout);
			// if (0 == timeout) timeout = 4;
			fCreateCommand(command, 0);
		}));

	parser.AddCommand(CommandParseInfo(
					"ArmMovement", {"A", "a"},
					[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
				parameters.resize(3);
				auto power = parameters[0];
				auto distance = parameters[1];
				auto timeout = parameters[2];
							Command *command = new ScriptArmMovement(power, distance, timeout);
				// if (0 == timeout) timeout = 4;
				fCreateCommand(command, 0);
		}));

	parser.AddCommand(CommandParseInfo(
						"ClawOpen", {"CO", "co"},
						[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
					parameters.resize(1);
					auto timeout = parameters[0];
								Command *command = new ScriptClawOpen(timeout);
					// if (0 == timeout) timeout = 4;
					fCreateCommand(command, 0);
			}));

	parser.AddCommand(CommandParseInfo(
						"ClawClose", {"CC", "cc"},
						[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
					parameters.resize(1);
					auto timeout = parameters[0];
								Command *command = new ScriptClawClose(timeout);
					// if (0 == timeout) timeout = 4;
					fCreateCommand(command, 0);
				}));

	parser.AddCommand(CommandParseInfo(
						"Rollers", {"R", "r"},
						[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
							parameters.resize(2);
						auto power = parameters[0];
						auto timeout = parameters[1];
									Command *command = new ScriptRollers(power, timeout);
						// if (0 == timeout) timeout = 4;
							if(power == 0) power = 0.5;
						fCreateCommand(command, 0);
					}));


	// Call IsValid to ensure that regular expressions
	// get built before the start of autonomous.
	parser.IsValid("S(0)");
}

START_ROBOT_CLASS(Robot);
