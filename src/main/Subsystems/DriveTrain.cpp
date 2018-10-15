#include "Subsystems/DriveTrain.h"
#include "Commands/CrabDrive.h"
#include "Commands/FieldCentric.h"
#include "Modules/Constants.h"
#include "Modules/Logger.h"
#include "Robot.h"
#include "RobotMap.h"
#include "Subsystems/EncoderConstants.h"
#include <cmath>
#include "Smartdashboard/smartdashboard.h"
#include "preferences.h"
#include "OI.h"
#include <iostream>

/*
#ifdef TESTSWERVE
#define MAXTURNS 3
#else
#define MAXTURNS 3000
#define SOFTTURNLIMIT 2
#endif
*/

#define FLS 0
#define FRS 1
#define RLS 2
#define RRS 3

const float TWISTSCALE = 0.6;

const float DEAD_ZONE = 0.15;

const float driveScale = 1;
//const float driveScale = 1;
// const double AVERAGE_VOLTAGE_BASE = EncoderConstants::HALF_TURN;

#define GYROP .01  // started at .01
#define GYROMAX 1.0

float lastx = 0.0;
float lasty = 0.0;
float lasttwist = 0.0;

// ==========================================================================

DriveTrain::DriveTrain() :
		frc::Subsystem("DriveTrain") {
	LOG("DriveTrain::ctor");

	unwinding = false;
	X = 0;
	Y = 0;

	FLOffset = 0;
	FROffset = 0;
	RLOffset = 0;
	RROffset = 0;

	FLInv = 1;
	FRInv = 1;
	RRInv = 1;
	RLInv = 1;

	lastx = 0.0;
	lasty = 0.0;
	lasttwist = 0.0;

	FLValue = 0;
	FRValue = 0;
	RLValue = 0;
	RRValue = 0;

	frontLeftDrive = RobotMap::driveTrainFrontLeftDrive;
	frontLeftSteer = RobotMap::driveTrainFrontLeftSteer;

	frontRightDrive = RobotMap::driveTrainFrontRightDrive;
	frontRightSteer = RobotMap::driveTrainFrontRightSteer;

	rearLeftDrive = RobotMap::driveTrainRearLeftDrive;
	rearLeftSteer = RobotMap::driveTrainRearLeftSteer;

	rearRightDrive = RobotMap::driveTrainRearRightDrive;
	rearRightSteer = RobotMap::driveTrainRearRightSteer;

	lidar = RobotMap::lidar;
	lidarDistance = 0;
}

// ==========================================================================

void DriveTrain::EnablePIDs(bool enable) {
	if (enable) {
		//frontLeftSteer->Enable();
		//frontRightSteer->Enable();
		//rearLeftSteer->Enable();
		//rearRightSteer->Enable();
	}
	else {
		//frontLeftSteer->Disable();
		//frontRightSteer->Disable();
		//rearLeftSteer->Disable();
		//rearRightSteer->Disable();
	}
}

// ==========================================================================

void DriveTrain::readLidar() {
	//lidar->ReadOnly(4, (unsigned char*)&lidarDistance);
}

// ==========================================================================

void DriveTrain::InitDefaultCommand() {
	SetDefaultCommand(new FieldCentric());
}

// ==========================================================================

void DriveTrain::SetWheelbase(float w, float x, float y) {
	//W = w;
	X = x;
	Y = y;
}

// ==========================================================================

void DriveTrain::SetOffsets(double FLOff, double FROff, double RLOff, double RROff) {
	FLOffset = FLOff;
	FROffset = FROff;
	RLOffset = RLOff;
	RROffset = RROff;
}

// ==========================================================================

void DriveTrain::PositionModeTwist(float desiredangle) {
	frontLeftSteer->Set(ControlMode::Position,CorrectSteerSetpoint(FLOffset + 0.625, frontLeftSteer));
	frontRightSteer->Set(ControlMode::Position,CorrectSteerSetpoint(FROffset - 0.625, frontRightSteer));
	rearLeftSteer->Set(ControlMode::Position,CorrectSteerSetpoint(RLOffset - 0.625, rearLeftSteer));
	rearRightSteer->Set(ControlMode::Position,CorrectSteerSetpoint(RROffset + 0.625, rearRightSteer));
}

// ==========================================================================

// keeps controls consistent regardless of rotation of robot
void DriveTrain::FieldCentricCrab(float twist, float y, float x, bool operatorControl) {
	float FWD = y;
	float STR = x;

	auto robotangle = Robot::gyroSub->PIDGet() * pi / 180;

	FWD = y * cos(robotangle) + x * sin(robotangle);
	STR = -y * sin(robotangle) + x * cos(robotangle);

	Crab((twist*0.65), FWD, STR, operatorControl); // twist * 0.65
}

// ==========================================================================

// attempts to keep robot square to the field as it drives
void DriveTrain::GyroCrab(float desiredangle, float y, float x, bool operatorControl) {
	auto robotangle = Robot::gyroSub->PIDGet();

	float twist;

    twist = desiredangle - robotangle;

	while (twist > 180.0)
		twist -= 360.0;
	while (twist < -180.0)
		twist += 360.0;

	twist = std::min(GYROMAX, std::max(-GYROMAX, twist * GYROP));
	Crab(twist, y, x, operatorControl);
}

// ==========================================================================

double DriveTrain::SnapToNearest90(double angle) {
	// Use abs to convert to int, add 45 to handle rounding, divide by 90 to get quadrant,
	// and multiply by multiplier to get nearest 90 with proper sign.
	auto multiplier = (angle >= 0) ? 90 : -90;
	//return multiplier * ((abs(angle) + 45) / 90);
	return multiplier * ((abs(angle) + 0) / 90);

}

// ==========================================================================

void DriveTrain::DriveDistanceStart(float twistAngle, float distance) {
	RobotMap::driveTrainFrontRightDrive->Set(ControlMode::Disabled, 0);
	RobotMap::driveTrainFrontLeftDrive->Set(ControlMode::Disabled, 0);
	RobotMap::driveTrainRearRightDrive->Set(ControlMode::Disabled, 0);
	RobotMap::driveTrainRearLeftDrive->Set(ControlMode::Disabled, 0);

	zeroDistanceEncoders();
	driveDistanceDelay = 0;

	driveDistanceMaintainAngle = Robot::gyroSub->PIDGet();
	driveDistanceMaintainAngle = SnapToNearest90(driveDistanceMaintainAngle);

	float twistAngleScaled = twistAngle / 360.0 * EncoderConstants::FULL_TURN;
	SetSteer(twistAngleScaled, twistAngleScaled, twistAngleScaled, twistAngleScaled);
/*
	float FWD = 0;
	float STR = 0;

	if (twistAngle == 0) {
		 FWD = 1;
		 STR = 0;
	}
	else if (twistAngle == 90) {
		 FWD = 0;
		 STR = 1;
	}
	else if (twistAngle == 180) {
		 FWD = -1;
		 STR = 0;
	}
	else if (twistAngle == 270) {
		 FWD = 0;
		 STR = -1;
	}

	auto AP = STR;
	auto BP = STR;
	auto CP = FWD;
	auto DP = FWD;

	float FLSetPoint = EncoderConstants::HALF_TURN;
	float FRSetPoint = EncoderConstants::HALF_TURN;
	float RLSetPoint = EncoderConstants::HALF_TURN;
	float RRSetPoint = EncoderConstants::HALF_TURN;

	if (DP != 0 || BP != 0) {
		FLSetPoint = (EncoderConstants::HALF_TURN + EncoderConstants::HALF_TURN / pi * atan2(BP, DP));
	}
	if (BP != 0 || CP != 0) {
		FRSetPoint = (EncoderConstants::HALF_TURN + EncoderConstants::HALF_TURN / pi * atan2(BP, CP));
	}
	if (AP != 0 || DP != 0) {
		RLSetPoint = (EncoderConstants::HALF_TURN + EncoderConstants::HALF_TURN / pi * atan2(AP, DP));
	}
	if (AP != 0 || CP != 0) {
		RRSetPoint = (EncoderConstants::HALF_TURN + EncoderConstants::HALF_TURN / pi * atan2(AP, CP));
	}

	SetSteer(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);
*/
	constexpr int ACCELERATION = 2500; // 2000
	constexpr int MAX_VELOCITY = 5000; // 4500

	RobotMap::driveTrainFrontRightDrive->ConfigMotionCruiseVelocity(MAX_VELOCITY, 10);
	RobotMap::driveTrainFrontRightDrive->ConfigMotionAcceleration(ACCELERATION, 10); //2000
	RobotMap::driveTrainFrontLeftDrive->ConfigMotionCruiseVelocity(MAX_VELOCITY, 10);
	RobotMap::driveTrainFrontLeftDrive->ConfigMotionAcceleration(ACCELERATION, 10); //2000
	RobotMap::driveTrainRearRightDrive->ConfigMotionCruiseVelocity(MAX_VELOCITY, 10);
	RobotMap::driveTrainRearRightDrive->ConfigMotionAcceleration(ACCELERATION, 10);
	RobotMap::driveTrainRearLeftDrive->ConfigMotionCruiseVelocity(MAX_VELOCITY, 10);
	RobotMap::driveTrainRearLeftDrive->ConfigMotionAcceleration(ACCELERATION, 10);
}

// ============================================================================

void DriveTrain::DriveDistanceExecute(float twistAngle, float distance) {
	float originalTwistAngle = twistAngle;

	if (driveDistanceDelay == 0 && GetClosedLoopOnTarget()) {
		float twistAngleScaled = twistAngle / 360.0 * EncoderConstants::FULL_TURN;
		SetSteer(twistAngleScaled, twistAngleScaled, twistAngleScaled, twistAngleScaled);
		driveDistanceDelay++;
		distance *= unitsPerFoot;
		std::cout << "distance: " << distance << std::endl;

		RobotMap::driveTrainFrontRightDrive->Set(ControlMode::MotionMagic, distance * FRInv);
		RobotMap::driveTrainFrontLeftDrive->Set(ControlMode::MotionMagic, -distance * FLInv);
		RobotMap::driveTrainRearRightDrive->Set(ControlMode::MotionMagic, distance * RRInv);
		RobotMap::driveTrainRearLeftDrive->Set(ControlMode::MotionMagic, -distance * RLInv);
	}
	else if (driveDistanceDelay > 0) {
		auto robotAngle = Robot::gyroSub->PIDGet();

		float twisterr = driveDistanceMaintainAngle - robotAngle;  //assume straight to field

		while (twisterr > 180.0) {
			twisterr -= 360.0;
		}
		while (twisterr < -180.0) {
			twisterr += 360.0;
		}

		//twisterr = std::min(GYROMAX, std::max(-GYROMAX, twisterr * GYROP));
		twistAngle += twisterr*2;
		//twist = twist/360.0 * EncoderConstants::FULL_TURN;
		printf("%f\n", twistAngle);
		twistAngle = twistAngle/360.0 * EncoderConstants::FULL_TURN;

		float originalTwistAngleScaled = originalTwistAngle / 360.0* EncoderConstants::FULL_TURN;

		while(originalTwistAngle < 0) {
			originalTwistAngle += 360;
		}
		while(originalTwistAngle > 360) {
			originalTwistAngle -= 360;
		}

		if (originalTwistAngle >= -45 && originalTwistAngle < 45) {
			SetSteerLockInv(twistAngle, twistAngle, originalTwistAngleScaled, originalTwistAngleScaled);
		}
		else if (originalTwistAngle >= 45 && originalTwistAngle < 135) {
			SetSteerLockInv(originalTwistAngleScaled, twistAngle, originalTwistAngleScaled, twistAngle);
		}
		else if (originalTwistAngle >= 135 && originalTwistAngle < 225) {
			SetSteerLockInv(originalTwistAngleScaled, originalTwistAngleScaled, twistAngle, twistAngle);
		}
		else if (originalTwistAngle >= 225 && originalTwistAngle < 315) {
			SetSteerLockInv(twistAngle, originalTwistAngleScaled, twistAngle, originalTwistAngleScaled);
		}
		else if (originalTwistAngle >= 315 && originalTwistAngle < 405) {
			SetSteerLockInv(twistAngle, twistAngle, originalTwistAngleScaled, originalTwistAngleScaled);
		}
	}
}

// ==============================================================================

void DriveTrain::DriveDistanceEnd() {
		/*RobotMap::driveTrainFrontRightDrive->ConfigMotionCruiseVelocity(0, 10);
		RobotMap::driveTrainFrontRightDrive->ConfigMotionAcceleration(0, 10);
		RobotMap::driveTrainFrontLeftDrive->ConfigMotionCruiseVelocity(0, 10);
		RobotMap::driveTrainFrontLeftDrive->ConfigMotionAcceleration(0, 10);
		RobotMap::driveTrainRearRightDrive->ConfigMotionCruiseVelocity(0, 10);
		RobotMap::driveTrainRearRightDrive->ConfigMotionAcceleration(0, 10);
		RobotMap::driveTrainRearLeftDrive->ConfigMotionCruiseVelocity(0, 10);
		RobotMap::driveTrainRearLeftDrive->ConfigMotionAcceleration(0, 10);
		RobotMap::driveTrainFrontRightDrive->Set(ControlMode::Disabled, 0);
		RobotMap::driveTrainFrontLeftDrive->Set(ControlMode::Disabled, 0);
		RobotMap::driveTrainRearRightDrive->Set(ControlMode::Disabled, 0);
		RobotMap::driveTrainRearLeftDrive->Set(ControlMode::Disabled, 0);*/

		//Crab(0, 0, 0, false);
}

// ===============================================================================

bool DriveTrain::WheelVelocityZero() {
	 return abs(RobotMap::driveTrainFrontLeftDrive->GetSelectedSensorVelocity(0)) < 250 &&
			abs(RobotMap::driveTrainFrontRightDrive->GetSelectedSensorVelocity(0)) < 250 &&
			abs(RobotMap::driveTrainRearLeftDrive->GetSelectedSensorVelocity(0)) < 250 &&
			abs(RobotMap::driveTrainRearRightDrive->GetSelectedSensorVelocity(0)) < 250;
 }

// ===============================================================================

constexpr int PID_TOLERANCE = 200;

bool DriveTrain::GetClosedLoopOnTarget() {
	return  abs(RobotMap::driveTrainFrontRightSteer->GetClosedLoopError(0)) < PID_TOLERANCE &&
			abs(RobotMap::driveTrainFrontLeftSteer->GetClosedLoopError(0)) < PID_TOLERANCE &&
			abs(RobotMap::driveTrainRearRightSteer->GetClosedLoopError(0)) < PID_TOLERANCE &&
			abs(RobotMap::driveTrainRearLeftSteer->GetClosedLoopError(0)) < PID_TOLERANCE;
}

// ===============================================================================

void DriveTrain::GyroRotate(float desiredangle, double power) {
	auto robotangle = Robot::gyroSub->PIDGet();

	float twist = desiredangle - robotangle;
	while (twist > 180.0) {
		twist -= 360.0;
	}
	while (twist < -180.0) {
		twist += 360.0;
	}

	twist = std::min(power, std::max(-power, twist * (0.2/10)));
	Crab(twist, 0, 0, false);
}

// ==========================================================================

void DriveTrain::Crab(float twist, float y, float x, bool operatorControl) {
	// stop PID loop if wires wrap.
	/*
	 if (unwinding || abs(frontRightPos->GetTurns()) > MAXTURNS ||
	 abs(rearRightPos->GetTurns()) > MAXTURNS ||
	 abs(frontLeftPos->GetTurns()) > MAXTURNS ||
	 abs(rearLeftPos->GetTurns()) > MAXTURNS) {
		 SetDriveSpeed(0, 0, 0, 0);
		 return;
	 }
	 */
	// this stores the direction of joystick when all axis last crossed into the
	// deadzone and keeps the wheels pointed that direction
	// this .1 should be kept the same as the deadzone in oi.cpp
	if (operatorControl && x == 0.0 && y == 0.0 && twist == 0.0) {
		if (fabs(lasty) > DEAD_ZONE || fabs(lastx) > DEAD_ZONE
				|| fabs(lasttwist) > DEAD_ZONE) {
			y = std::min(std::max(lasty, -DEAD_ZONE), DEAD_ZONE);
			x = std::min(std::max(lastx, -DEAD_ZONE), DEAD_ZONE);
			twist = std::min(std::max(lasttwist, -DEAD_ZONE), DEAD_ZONE);
		} else {
			y = .05;
			// default wheel position
		}
	} else	{
		lastx = x;
		lasty = y;
		lasttwist = twist;
	}

	if (operatorControl) {
		// scale for operator control
		x *= 1;
		y *= 1;
		float avg = (abs(x) + abs(y)) / 2;
		float scale = 1 - avg / 2;
		twist *= scale; // TWISTSCALE;
	}

	float FWD = y;
	float STR = x;

	auto radius = std::sqrt(pow(Y, 2) + pow(X, 2));

	auto AP = STR - twist * X / radius;
	auto BP = STR + twist * X / radius;
	auto CP = FWD - twist * Y / radius;
	auto DP = FWD + twist * Y / radius;

	float FLSetPoint = EncoderConstants::HALF_TURN;
	float FRSetPoint = EncoderConstants::HALF_TURN;
	float RLSetPoint = EncoderConstants::HALF_TURN;
	float RRSetPoint = EncoderConstants::HALF_TURN;

	if (DP != 0 || BP != 0) {
		FLSetPoint = (EncoderConstants::HALF_TURN
				+ EncoderConstants::HALF_TURN / pi * atan2(BP, DP));
	}
	if (BP != 0 || CP != 0) {
		FRSetPoint = (EncoderConstants::HALF_TURN
				+ EncoderConstants::HALF_TURN / pi * atan2(BP, CP));
	}
	if (AP != 0 || DP != 0) {
		RLSetPoint = (EncoderConstants::HALF_TURN
				+ EncoderConstants::HALF_TURN / pi * atan2(AP, DP));
	}
	if (AP != 0 || CP != 0) {
		RRSetPoint = (EncoderConstants::HALF_TURN
				+ EncoderConstants::HALF_TURN / pi * atan2(AP, CP));
	}

	SetSteer(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);

	double FL; // FL, distance from Front Left Wheel to the center of rotation
	double FR; // FR, distance from Front Right Wheel to the center of rotation
	double RL; // RL, distance from Rear Left Wheel to the center of rotation
	double RR; // RR, distance from Rear Right Wheel to the center of rotation

	FL = sqrt(pow(BP, 2) + pow(DP, 2));
	FR = sqrt(pow(BP, 2) + pow(CP, 2));
	RL = sqrt(pow(AP, 2) + pow(DP, 2));
	RR = sqrt(pow(AP, 2) + pow(CP, 2));

	// Solve for fastest wheel speed
	double speedarray[] = { fabs(FL), fabs(FR), fabs(RL), fabs(RR) };

	int length = 4;
	double maxspeed = speedarray[0];
	for (int i = 1; i < length; i++) {
		if (speedarray[i] > maxspeed) {
			maxspeed = speedarray[i];
		}
	}

	double FRRatio; // Ratio of Speed of Front Right wheel
	double FLRatio; // Ratio of Speed of Front Left wheel
	double RRRatio; // Ratio of Speed of Rear Right wheel
	double RLRatio; // Ratio of Speed of Rear Left wheel

	// Set ratios based on maximum wheel speed
	if (maxspeed > 1 || maxspeed < -1) {
		FLRatio = FL / maxspeed;
		FRRatio = FR / maxspeed;
		RLRatio = RL / maxspeed;
		RRRatio = RR / maxspeed;
	} else {
		FLRatio = FL;
		FRRatio = FR;
		RLRatio = RL;
		RRRatio = RR;
	}

	if (operatorControl && fabs(x) <= DEAD_ZONE && fabs(y) <= DEAD_ZONE
			&& fabs(twist) <= DEAD_ZONE) {
		FLRatio = 0.0;
		FRRatio = 0.0;
		RLRatio = 0.0;
		RRRatio = 0.0;
	}

	// Set drive speeds
	SetDriveSpeed(FLRatio, -FRRatio, RLRatio, -RRRatio);
}

// ==========================================================================

void setSteerSetpoint(float setpoint, WPI_TalonSRX* talon, double *inverse){
	float currentPosition = talon->GetSelectedSensorPosition(0) / 4096.0;
	int turns = trunc(currentPosition);
	float currentAngle = currentPosition - turns;
	//printf("currentPosition %f turns %d angle %f\n", currentPosition, turns, currentAngle);

	currentPosition *= EncoderConstants::FULL_TURN;
	turns *= EncoderConstants::FULL_TURN;
	currentAngle *= EncoderConstants::FULL_TURN;

	float angleOptions[6];
	angleOptions[0] = turns - EncoderConstants::FULL_TURN + setpoint;
	angleOptions[1] = turns - EncoderConstants::FULL_TURN + setpoint + EncoderConstants::HALF_TURN;
	angleOptions[2] = turns + setpoint;
	angleOptions[3] = turns + setpoint + EncoderConstants::HALF_TURN;
	angleOptions[4] = turns + EncoderConstants::FULL_TURN + setpoint;
	angleOptions[5] = turns + EncoderConstants::FULL_TURN + setpoint + EncoderConstants::HALF_TURN;

	float minMove = fabs(currentPosition - angleOptions[0]);
	int minI = 0;
	for (int i = 1; i < 6; i++){
		if (fabs(currentPosition - angleOptions[i]) < minMove){
			minMove = fabs(currentPosition - angleOptions[i]);
			minI = i;
		}
	}
	talon->Set(ControlMode::Position,angleOptions[minI]/EncoderConstants::FULL_TURN * 4096);

	//*inverse = cos((minMove / EncoderConstants::FULL_TURN) * 2 * 3.141);
	//if (minI % 2)
	//	*inverse *= -1;
	//else
	//	*inverse *= 1;
	if (minI % 2)
		*inverse = -1;
	else
		*inverse = 1;

}

// ==========================================================================

void setSteerSetpointLockInv(float setpoint, WPI_TalonSRX* talon, double *inverse) {
	float currentPosition = talon->GetSelectedSensorPosition(0) / 4096.0;
	int turns = trunc(currentPosition);
	float currentAngle = currentPosition - turns;
	//printf("currentPosition %f turns %d angle %f\n", currentPosition, turns, currentAngle);

	currentPosition *= EncoderConstants::FULL_TURN;
	turns *= EncoderConstants::FULL_TURN;
	currentAngle *= EncoderConstants::FULL_TURN;

	float angleOptions[3];

	if (*inverse == -1) {
		angleOptions[0] = turns - EncoderConstants::FULL_TURN + setpoint + EncoderConstants::HALF_TURN;
		angleOptions[1] = turns + setpoint + EncoderConstants::HALF_TURN;
		angleOptions[2] = turns + EncoderConstants::FULL_TURN + setpoint + EncoderConstants::HALF_TURN;
	}
	else {
		angleOptions[0] = turns - EncoderConstants::FULL_TURN + setpoint;
		angleOptions[1] = turns + setpoint;
		angleOptions[2] = turns + EncoderConstants::FULL_TURN + setpoint;
	}

	float minMove = fabs(currentPosition - angleOptions[0]);
	int minI = 0;
	for (int i = 1; i < 3; i++) {
		if (fabs(currentPosition - angleOptions[i]) < minMove) {
			minMove = fabs(currentPosition - angleOptions[i]);
			minI = i;
		}
	}
	talon->Set(ControlMode::Position,angleOptions[minI]/EncoderConstants::FULL_TURN * 4096);
}

// ==========================================================================

void DriveTrain::SetSteer(float FLSetPoint, float FRSetPoint, float RLSetPoint, float RRSetPoint) {
	FLSetPoint = -FLSetPoint;
	FRSetPoint = -FRSetPoint;
	RLSetPoint = -RLSetPoint;
	RRSetPoint = -RRSetPoint;
	//LogSettings(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);

	setSteerSetpoint(FLSetPoint + FLOffset, frontLeftSteer, &FLInv);
	setSteerSetpoint(FRSetPoint + FROffset, frontRightSteer, &FRInv);
	setSteerSetpoint(RLSetPoint + RLOffset, rearLeftSteer, &RLInv);
	setSteerSetpoint(RRSetPoint + RROffset, rearRightSteer, &RRInv);
}

// ==========================================================================

void DriveTrain::SetSteerLockInv(float FLSetPoint, float FRSetPoint, float RLSetPoint, float RRSetPoint) {
	FLSetPoint = -FLSetPoint;
	FRSetPoint = -FRSetPoint;
	RLSetPoint = -RLSetPoint;
	RRSetPoint = -RRSetPoint;
	//LogSettings(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);

	setSteerSetpointLockInv(FLSetPoint + FLOffset, frontLeftSteer, &FLInv);
	setSteerSetpointLockInv(FRSetPoint + FROffset, frontRightSteer, &FRInv);
	setSteerSetpointLockInv(RLSetPoint + RLOffset, rearLeftSteer, &RLInv);
	setSteerSetpointLockInv(RRSetPoint + RROffset, rearRightSteer, &RRInv);
}

// ==========================================================================

void DriveTrain::SetDriveSpeed(float FLSpeed, float FRSpeed, float RLSpeed, float RRSpeed) {
	//LogSettings(FLSpeed, FRSpeed, RLSpeed, RRSpeed);
	if (RobotMap::SpeedControl == 0) {  //percentvbus
		frontLeftDrive->Set(ControlMode::PercentOutput,FLSpeed * FLInv * driveScale);
		frontRightDrive->Set(ControlMode::PercentOutput,FRSpeed * FRInv * driveScale);
		rearLeftDrive->Set(ControlMode::PercentOutput,RLSpeed * RLInv * driveScale);
		rearRightDrive->Set(ControlMode::PercentOutput,RRSpeed * RRInv * driveScale);
	} else if (RobotMap::SpeedControl == 1) { //velocity
		frontLeftDrive->Set(ControlMode::Velocity  ,FLSpeed * FLInv * 7000);  // experimentally determined free drivetrain speed pulses/.1s
		frontRightDrive->Set(ControlMode::Velocity,FRSpeed * FRInv * 7000);
		rearLeftDrive->Set(ControlMode::Velocity,RLSpeed * RLInv * 7000);
		rearRightDrive->Set(ControlMode::Velocity,RRSpeed * RRInv * 7000);
	}
	else if (RobotMap::SpeedControl == 2) { //position
	}
}

// ==========================================================================

void DriveTrain::Lock() {
	SetSteer(3.0, 1.5, 3.0, 1.5);
	SetDriveSpeed(0, 0, 0, 0);
}

// ==========================================================================

void DriveTrain::SideLock() {
	SetSteer(2.0, 0.75, 3.25, 4.5);
	SetDriveSpeed(0, 0, 0, 0);
}

// ==========================================================================

void DriveTrain::updateDistanceEncoders() {

	FLValue = frontLeftDrive->GetSelectedSensorPosition(0);
	FRValue = frontRightDrive->GetSelectedSensorPosition(0);
	RLValue = rearLeftDrive->GetSelectedSensorPosition(0);
	RRValue = rearRightDrive->GetSelectedSensorPosition(0);
}

// ==========================================================================

double DriveTrain::getDistanceEncodersValues() {
	double average = (fabs(FLValue) + fabs(FRValue) + fabs(RLValue) + fabs(RRValue)) / 4;
	average = average/ unitsPerFoot;
	return average;
}

// ==========================================================================

void DriveTrain::zeroDistanceEncoders() {
	frontLeftDrive->SetSelectedSensorPosition(0,0,10);
	frontRightDrive->SetSelectedSensorPosition(0,0,10);
	rearLeftDrive->SetSelectedSensorPosition(0,0,10);
	rearRightDrive->SetSelectedSensorPosition(0,0,10);
}

// ==========================================================================

void DriveTrain::zeroSteeringEncoders() {
	frontLeftSteer->SetSelectedSensorPosition(0,0,10);
	frontRightSteer->SetSelectedSensorPosition(0,0,10);
	rearLeftSteer->SetSelectedSensorPosition(0,0,10);
	rearRightSteer->SetSelectedSensorPosition(0,0,10);

	FLValue = 0;
	FRValue = 0;
	RLValue = 0;
	RRValue = 0;
}

// ==========================================================================

void DriveTrain::setWheelOffsets() {
	// Get the current steering positions from the drive train.
	/*
	 auto FLPosition = Robot::driveTrain->frontLeftPos->GetRawAngle();
	 auto FRPosition = Robot::driveTrain->frontRightPos->GetRawAngle();
	 auto RLPosition = Robot::driveTrain->rearLeftPos->GetRawAngle();
	 auto RRPosition = Robot::driveTrain->rearRightPos->GetRawAngle();
	 auto ArmPosition = Robot::armSub->GetRawPosition();
	 */
	auto FLPosition = getTalonPosition(frontLeftSteer);
	auto FRPosition = getTalonPosition(frontRightSteer);
	auto RLPosition = getTalonPosition(rearLeftSteer);
	auto RRPosition = getTalonPosition(rearRightSteer);
	//LogSettings(FLPosition, FRPosition, RLPosition, RRPosition);

	// Save the positions to Preferences.
	auto prefs = frc::Preferences::GetInstance();
	prefs->PutDouble(Constants::FL_POS_NAME, FLPosition);
	prefs->PutDouble(Constants::FR_POS_NAME, FRPosition);
	prefs->PutDouble(Constants::RL_POS_NAME, RLPosition);
	prefs->PutDouble(Constants::RR_POS_NAME, RRPosition);
	//prefs->PutDouble(Constants::ARM_POSITION_NAME, ArmPosition);

	// Set the drive train positions.
	SetOffsets(FLPosition, FRPosition, RLPosition, RRPosition);
	//Robot::armSub->SetOffset(ArmPosition);
}

// ==========================================================================

void DriveTrain::loadWheelOffsets() {
	LOG("DriveTrainSettings::LoadSettings");

	// Load the positions from Preferences.
	auto prefs = frc::Preferences::GetInstance();
	auto FLPosition = prefs->GetDouble(Constants::FL_POS_NAME,
			Constants::FL_POS_DEFAULT);
	auto FRPosition = prefs->GetDouble(Constants::FR_POS_NAME,
			Constants::FR_POS_DEFAULT);
	auto RLPosition = prefs->GetDouble(Constants::RL_POS_NAME,
			Constants::RL_POS_DEFAULT);
	auto RRPosition = prefs->GetDouble(Constants::RR_POS_NAME,
			Constants::RR_POS_DEFAULT);

	//LogSettings(FLPosition, FRPosition, RLPosition, RRPosition);

	// Set the drive train positions.
	SetOffsets(FLPosition, FRPosition, RLPosition, RRPosition);
}

// ==========================================================================

void DriveTrain::LogSettings(double fl, double fr, double rl, double rr) {
	char sz[256];
	sprintf(sz, "Wheel Positions: FL %f, FR %f, RL %f, RR %f", fl, fr, rl, rr);
	LOG(sz);
}

// ==========================================================================

void DriveTrain::Dashboard() {
	/*frc::SmartDashboard::PutNumber("Steering Motor Encoder FL",
			frontLeftSteer->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("Steering Motor Encoder FR",
			frontRightSteer->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("Steering Motor Encoder RL",
			rearLeftSteer->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("Steering Motor Encoder RR",
			rearRightSteer->GetSelectedSensorPosition(0));

	frc::SmartDashboard::PutNumber("FR Setpoint", frontRightSteer->GetClosedLoopTarget(0));
	frc::SmartDashboard::PutNumber("FL Setpoint", frontLeftSteer->GetClosedLoopTarget(0));
	frc::SmartDashboard::PutNumber("RR Setpoint", rearRightSteer->GetClosedLoopTarget(0));
	frc::SmartDashboard::PutNumber("RL Setpoint", rearLeftSteer->GetClosedLoopTarget(0));*/
}

// ==========================================================================

void DriveTrain::CrabInit() {
	//frontLeftSteer->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
	//frontRightSteer->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
	//rearLeftSteer->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
	//rearRightSteer->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
	//frontLeftSteer->Enable();
	//frontRightSteer->Enable();
	//rearLeftSteer->Enable();
	//rearRightSteer->Enable();
	//frontLeftSteer->SetControlMode(CANSpeedController::kPosition);
	//frontRightSteer->SetControlMode(CANSpeedController::kPosition);
	//rearLeftSteer->SetControlMode(CANSpeedController::kPosition);
	//rearRightSteer->SetControlMode(CANSpeedController::kPosition);
}

// ==========================================================================

void DriveTrain::SetWheelsStraight() {
	frontLeftSteer->Set(ControlMode::Position,FLOffset);
	frontRightSteer->Set(ControlMode::Position,FROffset);
	rearLeftSteer->Set(ControlMode::Position,FROffset);
	rearRightSteer->Set(ControlMode::Position,FROffset);
}

// ==========================================================================

void DriveTrain::ArcadeDriveMode(float x, float y) {
	float leftMotorOutput;
	float rightMotorOutput;

	if (y > 0.0) {
		if (x > 0.0) {
			leftMotorOutput = y - x;
			rightMotorOutput = std::max(y, x);
		}
		else {
			leftMotorOutput = std::max(y, -x);
			rightMotorOutput = y + x;
		}
	}
	else {
		if (x > 0.0) {
			leftMotorOutput = -std::max(-y, x);
			rightMotorOutput = y + x;
		}
		else {
			leftMotorOutput = y - x;
			rightMotorOutput = -std::max(-y, -x);
		}
	}

	SetDriveSpeed(leftMotorOutput, rightMotorOutput, leftMotorOutput, rightMotorOutput);
}

// ==========================================================================

void DriveTrain::disableSpeedControl() {
	//frontLeftDrive->SetControlMode(CANSpeedController::kPercentVbus);
	//frontRightDrive->SetControlMode(CANSpeedController::kPercentVbus);
	//rearLeftDrive->SetControlMode(CANSpeedController::kPercentVbus);
	//rearRightDrive->SetControlMode(CANSpeedController::kPercentVbus);
	RobotMap::SpeedControl = 0;
}

// ==========================================================================

void DriveTrain::enableSpeedControl() {
	frontLeftDrive->SelectProfileSlot(0,0);
	frontRightDrive->SelectProfileSlot(0,0);
	rearLeftDrive->SelectProfileSlot(0,0);
	rearRightDrive->SelectProfileSlot(0,0);
	frontLeftDrive->ConfigPeakOutputForward(1,100);
	frontLeftDrive->ConfigPeakOutputReverse(-1,100);
	frontRightDrive->ConfigPeakOutputForward(1,100);
	frontRightDrive->ConfigPeakOutputReverse(-1,100);
	rearLeftDrive->ConfigPeakOutputForward(1,100);
	rearLeftDrive->ConfigPeakOutputReverse(-1,100);
	rearRightDrive->ConfigPeakOutputForward(1,100);
	rearRightDrive->ConfigPeakOutputReverse(-1,100);
	//frontLeftDrive->SetControlMode(CANSpeedController::kSpeed);
	//frontRightDrive->SetControlMode(CANSpeedController::kSpeed);
	//rearLeftDrive->SetControlMode(CANSpeedController::kSpeed);
	//rearRightDrive->SetControlMode(CANSpeedController::kSpeed);
	RobotMap::SpeedControl = 1;
}

// ==========================================================================

void DriveTrain::enablePositionControl() {
	frontLeftDrive->SelectProfileSlot(1,0);
	frontRightDrive->SelectProfileSlot(1,0);
	rearLeftDrive->SelectProfileSlot(1,0);
	rearRightDrive->SelectProfileSlot(1,0);
	frontLeftDrive->ConfigPeakOutputForward(.33,100);
	frontLeftDrive->ConfigPeakOutputReverse(-.33,100);
	frontRightDrive->ConfigPeakOutputForward(.33,100);
	frontRightDrive->ConfigPeakOutputReverse(-.33,100);
	rearLeftDrive->ConfigPeakOutputForward(.33,100);
	rearLeftDrive->ConfigPeakOutputReverse(-.33,100);
	rearRightDrive->ConfigPeakOutputForward(.33,100);
	rearRightDrive->ConfigPeakOutputReverse(-.33,100);
	//frontLeftDrive->SetControlMode(CANSpeedController::kPosition);
	//frontRightDrive->SetControlMode(CANSpeedController::kPosition);
	//rearLeftDrive->SetControlMode(CANSpeedController::kPosition);
	//rearRightDrive->SetControlMode(CANSpeedController::kPosition);
	RobotMap::SpeedControl = 2;
}

// ==========================================================================

void DriveTrain::enableSteeringPID() {
	//frontLeftSteer->Enable();
	//frontRightSteer->Enable();
	//rearLeftSteer->Enable();
	//rearRightSteer->Enable();
}

// ==========================================================================

double DriveTrain::getTalonPosition(WPI_TalonSRX* talon) {
	float currentPosition = talon->GetSelectedSensorPosition(0) / 4096.0;
	int turns = trunc(currentPosition);
	float currentAngle = currentPosition - turns;
    return currentAngle * EncoderConstants::FULL_TURN;
}

// ==========================================================================

bool DriveTrain::unwind() { // float y, float x){
	//frontLeftSteer->Disable();
	//frontRightSteer->Disable();
	//rearLeftSteer->Disable();
	//rearRightSteer->Disable();

	frontLeftSteer->Config_kP(0,0.4,100);
	frontRightSteer->Config_kP(0,0.4,100);
	rearLeftSteer->Config_kP(0,0.4,100);
	rearRightSteer->Config_kP(0,0.4,100);

	frontLeftSteer->ConfigPeakOutputForward(.5,100);
	frontLeftSteer->ConfigPeakOutputReverse(-.5,100);
	frontRightSteer->ConfigPeakOutputForward(.5,100);
	frontRightSteer->ConfigPeakOutputReverse(-.5,100);
	rearLeftSteer->ConfigPeakOutputForward(.5,100);
	rearLeftSteer->ConfigPeakOutputReverse(-.5,100);
	rearRightSteer->ConfigPeakOutputForward(.5,100);
	rearRightSteer->ConfigPeakOutputReverse(-.5,100);

	frontLeftSteer->Set(ControlMode::Position,FLOffset / EncoderConstants::FULL_TURN);
	frontRightSteer->Set(ControlMode::Position,FROffset / EncoderConstants::FULL_TURN);
	rearLeftSteer->Set(ControlMode::Position,RLOffset / EncoderConstants::FULL_TURN);
	rearRightSteer->Set(ControlMode::Position,RROffset / EncoderConstants::FULL_TURN);

	unwinding = true;
	return true;
}

// ==========================================================================

void DriveTrain::doneunwind() {
	unwinding = 0;
}

// ==========================================================================

double DriveTrain::CorrectSteerSetpoint(double setpoint, WPI_TalonSRX* talon) {
	if (setpoint < 0) {
		return (setpoint + EncoderConstants::FULL_TURN)/EncoderConstants::FULL_TURN;
	}
	else if (setpoint > EncoderConstants::FULL_TURN) {
		return (setpoint - EncoderConstants::FULL_TURN)/EncoderConstants::FULL_TURN;
	}
	else if (setpoint == EncoderConstants::FULL_TURN) {
		return 0;
	}
	else {
		return setpoint/EncoderConstants::FULL_TURN;
	}

	/*
	if (setpoint < 0) {
		setpoint = (setpoint + EncoderConstants::FULL_TURN) / EncoderConstants::FULL_TURN;
	} else if (setpoint > EncoderConstants::FULL_TURN) {
		setpoint = (setpoint - EncoderConstants::FULL_TURN) / EncoderConstants::FULL_TURN;
	}

	float currentPosition = talon->GetSelectedSensorPosition(0);

	if (setpoint < 0.25 && currentPosition - trunc(currentPosition) > 0.75) {
		return (trunc(currentPosition) + setpoint + 1);
	} else if (setpoint > 0.75 && currentPosition - trunc(currentPosition) < 0.25) {
		return (trunc(currentPosition) + setpoint - 1);
	} else {
		return trunc(currentPosition) + setpoint;
	}
	*/
	/*
	 int newSetpoint = (trunc(currentPosition)+setpoint);

	 while (newSetpoint - currentPosition < -EncoderConstants::FULL_TURN){
	 newSetpoint -= EncoderConstants::FULL_TURN;
	 }
	 while (newSetpoint - currentPosition > EncoderConstants::FULL_TURN){
	 newSetpoint += EncoderConstants::FULL_TURN;
	 }

	 return newSetpoint / EncoderConstants::FULL_TURN;


	 if (setpoint < 0) {
	 return (setpoint + EncoderConstants::FULL_TURN)/EncoderConstants::FULL_TURN;
	 }
	 else if (setpoint > EncoderConstants::FULL_TURN) {
	 return (setpoint - EncoderConstants::FULL_TURN)/EncoderConstants::FULL_TURN;
	 }
	 else if (setpoint == EncoderConstants::FULL_TURN) {
	 return 0;
	 }
	 else {
	 return setpoint/5;
	 }*/
}

// ==========================================================================

//Old SetSteer function
/*
void DriveTrain::SetSteer(float FLSetPoint, float FRSetPoint,
		float RLSetPoint, float RRSetPoint) {
	FLSetPoint = -FLSetPoint;
	FRSetPoint = -FRSetPoint;
	RLSetPoint = -RLSetPoint;
	RRSetPoint = -RRSetPoint;

	setSteerSetpoint(FLSetPoint + FLOffset, frontLeftSteer, &FLInv);
	setSteerSetpoint(FRSetPoint + FROffset, frontRightSteer, &FRInv);
	setSteerSetpoint(RLSetPoint + RLOffset, rearLeftSteer, &RLInv);
	setSteerSetpoint(RRSetPoint + RROffset, rearRightSteer, &RRInv);

	return;

	//////////////////////////////////
	// Front Left Wheel
	//////////////////////////////////
		if (frontLeftPos->GetTurns() > SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(FLSetPoint + FLOffset - frontLeftPos->GetAngle()) > EncoderConstants::HALF_TURN) {
	 frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint + FLOffset));
	 FLInv = 1;
	 }
	 else {
	 frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint + FLOffset - EncoderConstants::HALF_TURN));
	 FLInv = -1;
	 }
	 }
	 else if (frontLeftPos->GetTurns() < -SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(FLSetPoint + FLOffset - frontLeftPos->GetAngle()) < EncoderConstants::HALF_TURN) {
	 frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint + FLOffset));
	 FLInv = 1;
	 }
	 else {
	 frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint + FLOffset - EncoderConstants::HALF_TURN));
	 FLInv = -1;
	 }
	 }
	 else {
	// Default rotation logic
	if (fabs(FLSetPoint + FLOffset - getTalonPosition(frontLeftSteer))
			< EncoderConstants::QUARTER_TURN
			|| fabs(FLSetPoint + FLOffset - getTalonPosition(frontLeftSteer))
					> EncoderConstants::THREEQUARTER_TURN) {
		frontLeftSteer->SetSetpoint(
				CorrectSteerSetpoint(FLSetPoint + FLOffset, frontLeftSteer));
		FLInv = 1;
	} else {
		frontLeftSteer->SetSetpoint(
				CorrectSteerSetpoint(
						FLSetPoint + FLOffset - EncoderConstants::HALF_TURN,
						frontLeftSteer));
		FLInv = -1;
	}
	//}

	//////////////////////////////////
	// Front Right Wheel
	//////////////////////////////////
	if (frontRightPos->GetTurns() > SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(FRSetPoint + FROffset - frontRightPos->GetAngle()) > EncoderConstants::HALF_TURN) {
	 frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint + FROffset));
	 FRInv = 1;
	 }
	 else {
	 frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint + FROffset - EncoderConstants::HALF_TURN));
	 FRInv = -1;
	 }
	 }
	 else if (frontRightPos->GetTurns() < -SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(FRSetPoint + FROffset - frontRightPos->GetAngle()) < EncoderConstants::HALF_TURN) {
	 frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint + FROffset));
	 FRInv = 1;
	 }
	 else {
	 frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint + FROffset - EncoderConstants::HALF_TURN));
	 FRInv = -1;
	 }
	 }
	 else {
	// default rotation logic
	if (fabs(FRSetPoint + FROffset - getTalonPosition(frontRightSteer))
			< EncoderConstants::QUARTER_TURN
			|| fabs(FRSetPoint + FROffset - getTalonPosition(frontRightSteer))
					> EncoderConstants::THREEQUARTER_TURN) {
		frontRightSteer->SetSetpoint(
				CorrectSteerSetpoint(FRSetPoint + FROffset, frontRightSteer));
		FRInv = 1;
	} else {
		frontRightSteer->SetSetpoint(
				CorrectSteerSetpoint(
						FRSetPoint + FROffset - EncoderConstants::HALF_TURN,
						frontRightSteer));
		FRInv = -1;
	}
	//}

	//////////////////////////////////
	// Rear Left Wheel
	//////////////////////////////////
	if (rearLeftPos->GetTurns() > SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(RLSetPoint + RLOffset - rearLeftPos->GetAngle()) > EncoderConstants::HALF_TURN) {
	 rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint + RLOffset));
	 RLInv = 1;
	 }
	 else {
	 rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint + RLOffset - EncoderConstants::HALF_TURN));
	 RLInv = -1;
	 }
	 }
	 else if (rearLeftPos->GetTurns() < -SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(RLSetPoint + RLOffset - rearLeftPos->GetAngle()) < EncoderConstants::HALF_TURN) {
	 rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint + RLOffset));
	 RLInv = 1;
	 }
	 else {
	 rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint + RLOffset - EncoderConstants::HALF_TURN));
	 RLInv = -1;
	 }
	 }
	 else {
	// default rotation logic
	if (fabs(RLSetPoint + RLOffset - getTalonPosition(rearLeftSteer))
			< EncoderConstants::QUARTER_TURN
			|| fabs(RLSetPoint + RLOffset - getTalonPosition(rearLeftSteer))
					> EncoderConstants::THREEQUARTER_TURN) {
		rearLeftSteer->SetSetpoint(
				CorrectSteerSetpoint(RLSetPoint + RLOffset, rearLeftSteer));
		RLInv = 1;
	} else {
		rearLeftSteer->SetSetpoint(
				CorrectSteerSetpoint(
						RLSetPoint + RLOffset - EncoderConstants::HALF_TURN,
						rearLeftSteer));
		RLInv = -1;
	}
	//}

	//////////////////////////////////
	// Rear Right Wheel
	//////////////////////////////////
	if (rearRightPos->GetTurns() > SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(RRSetPoint + RROffset - rearRightPos->GetAngle()) > EncoderConstants::HALF_TURN) {
	 rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint + RROffset));
	 RRInv = 1;
	 }
	 else {
	 rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint + RROffset - EncoderConstants::HALF_TURN));
	 RRInv = -1;
	 }
	 }
	 else if (rearRightPos->GetTurns() < -SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(RRSetPoint + RROffset - rearRightPos->GetAngle()) < EncoderConstants::HALF_TURN) {
	 rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint + RROffset));
	 RRInv = 1;
	 }
	 else {
	 rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint + RROffset - EncoderConstants::HALF_TURN));
	 RRInv = -1;
	 }
	 }
	 else {
	// default rotation logic
	if (fabs(RRSetPoint + RROffset - getTalonPosition(rearRightSteer))
			< EncoderConstants::QUARTER_TURN
			|| fabs(RRSetPoint + RROffset - getTalonPosition(rearRightSteer))
					> EncoderConstants::THREEQUARTER_TURN) {
		rearRightSteer->SetSetpoint(
				CorrectSteerSetpoint(RRSetPoint + RROffset, rearRightSteer));
		RRInv = 1;
	} else {
		rearRightSteer->SetSetpoint(
				CorrectSteerSetpoint(
						RRSetPoint + RROffset - EncoderConstants::HALF_TURN,
						rearRightSteer));
		RRInv = -1;
	}
	//}
}
*/
