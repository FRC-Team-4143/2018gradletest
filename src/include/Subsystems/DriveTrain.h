#pragma once

#include <Commands/Subsystem.h>
#include "Subsystems/GyroSub.h"
#include "ctre/phoenix.h"
#include "I2C.h"

// ==========================================================================

class DriveTrain : public frc::Subsystem {
private:
	static constexpr double pi = 3.141592653589793238462643383;

	bool unwinding;

	static constexpr double feetPerRev = 4*pi/12;
	static constexpr double unitsPerFoot = 4096/feetPerRev ;

	// Steering Variables
	double X; // X is the distance form the mid-point of the back wheels to the mid-point of the front wheels
	double Y; // Y is the distance from the mid-point of one front wheel to the mid-point of the other front wheel
	//double Z; // Z, distance form A * X to center of rotation

	double FLOffset; // Front left wheel position
	double FROffset; // Front right wheel position
	double RLOffset; // Rear left wheel position
	double RROffset; // Rear right wheel position

	// Inversion Variables
	double FLInv; // Direction of front left drive motor
	double FRInv; // Direction of front right drive motor
	double RLInv; // Direction of right left drive motor
	double RRInv; // Direction of right rear drive motor

	float lastx;
	float lasty;
	float lasttwist;

	int driveDistanceDelay;
	double driveDistanceMaintainAngle;

	double FLValue; // Current value measured from front left drive encoder
	double FRValue; // Current value measured from front right drive encoder
	double RLValue; // Current value measured from rear left drive encoder
	double RRValue; // Current value measured from rear right drive encoder

	WPI_TalonSRX* frontLeftDrive;			// Speed controller for front left drive motor
	WPI_TalonSRX* frontLeftSteer;			// Speed controller for front left steering motor

	WPI_TalonSRX* frontRightDrive;			// Speed controller for front right drive motor
	WPI_TalonSRX* frontRightSteer;			// Speed controller for front right steering motor

	WPI_TalonSRX* rearLeftDrive;			// Speed controller for rear left drive motor
	WPI_TalonSRX* rearLeftSteer;			// Speed controller for rear left steering motor

	WPI_TalonSRX* rearRightDrive;			// Speed controller for rear right drive motor
	WPI_TalonSRX* rearRightSteer;			// Speed controller for rear right steering motor

	frc::I2C* lidar;

	double CorrectSteerSetpoint(double setpoint, WPI_TalonSRX* talon);
	void SetSteer(float FLSetPoint, float FRSetPoint, float RLSetPoint, float RRSetPoint);
	void SetSteerSetpoint(float setpoint, WPI_TalonSRX* talon, int *inverse);
	void SetSteerLockInv(float FLSetPoint, float FRSetPoint, float RLSetPoint, float RRSetPoint);
	void SetSteerSetpointLockInv(float setpoint, WPI_TalonSRX* talon, int *inverse);
	void SetDriveSpeed(float FLSpeed, float FRSpeed, float RLSpeed, float RRSpeed);
	void CurrentLimit();
	double getTalonPosition(WPI_TalonSRX* talon);

	static double SnapToNearest90(double angle);

public:
	DriveTrain();

	//float DEAD_ZONE;
	void readLidar();
	unsigned long lidarDistance;
	void EnablePIDs(bool enable);

	void SetWheelbase(float w, float x, float y); // sets dimensions of drive train
	void SetMaxSpeed(float MaxSpeed); // sets the max speed of drive motors
	void SetOffsets(double FLOff, double FROff, double RLOff, double RROff); // sets wheels to correct positions
	void InitDefaultCommand(); // sets default command
	void Crab(float twist, float y, float x, bool operatorControl); // default driving command
	void Lock();
	void SideLock(); // locks steering
	void DriveDistanceStart(float twistAngle, float distance);
	void DriveDistanceExecute(float twistAngle, float distance);
	void DriveDistanceEnd();
	bool WheelVelocityZero();
	bool GetClosedLoopOnTarget();
	bool unwind();     // float y, float x); //unwinds all wheels
	void doneunwind(); // called when unwinding is finished
	void GyroCrab(float desiredangle, float y, float x, bool operatorControl); // drive using gyro values
	void GyroRotate(float desiredangle, double power);
	void FieldCentricCrab(float twist, float y, float x, bool operatorControl); // drive at 90 degree angle to field
	void PositionModeTwist(float desiredangle);
	void updateDistanceEncoders(); // updates the values of drive train variables with distance encoder values
	double getDistanceEncodersValues(); // returns average value from driving encoders
	void zeroDistanceEncoders(); // Reset driving encoders to 0
	void zeroSteeringEncoders(); // Reset steering encoders to 0
	void setWheelOffsets();
	void loadWheelOffsets();
	void LogSettings(double fl, double fr, double rl, double rr);
	void Dashboard();
	void CrabInit();
	void SetWheelsStraight();
	void ArcadeDriveMode(float y, float x);
	void disableSpeedControl();
	void enableSpeedControl();
	void enablePositionControl();
	void enableSteeringPID();
};

// ==========================================================================
