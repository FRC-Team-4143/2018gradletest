#include "RobotMap.h"
#include <LiveWindow/LiveWindow.h>
#include "ctre/phoenix.h"
#include "PowerDistributionPanel.h"
#include "I2C.h"
#include "AnalogInput.h"
#include "SPI.h"

int RobotMap::SpeedControl = 0;

frc::PowerDistributionPanel* RobotMap::pdp = nullptr;

WPI_TalonSRX*          RobotMap::driveTrainFrontLeftDrive = nullptr;
WPI_TalonSRX*          RobotMap::driveTrainFrontLeftSteer = nullptr;

WPI_TalonSRX*          RobotMap::driveTrainFrontRightDrive = nullptr;
WPI_TalonSRX*          RobotMap::driveTrainFrontRightSteer = nullptr;

WPI_TalonSRX*          RobotMap::driveTrainRearLeftDrive = nullptr;
WPI_TalonSRX*          RobotMap::driveTrainRearLeftSteer = nullptr;

WPI_TalonSRX*          RobotMap::driveTrainRearRightDrive = nullptr;
WPI_TalonSRX*          RobotMap::driveTrainRearRightSteer = nullptr;

frc::I2C* RobotMap::i2c = nullptr;
frc::I2C* RobotMap::lidar = nullptr;

frc::AnalogInput* RobotMap::sonar = nullptr;

//SerialPort* RobotMap::serialPort = nullptr;
//SerialPort* RobotMap::serialPort1 = nullptr;
//SerialPort* RobotMap::serialPort2 = nullptr;

WPI_TalonSRX* RobotMap::climbingMotor = nullptr;
WPI_TalonSRX* RobotMap::elevatorMotor = nullptr;
WPI_TalonSRX* RobotMap::armMotor = nullptr;
WPI_TalonSRX* RobotMap::clawMotor = nullptr;
WPI_TalonSRX* RobotMap::rollerMotor = nullptr;

AHRS* RobotMap::imu = nullptr;

#define CONTINUOUS true
#define P 1.0 //1.1 // 0.7
#define I 0.005
#define D 0.1 // 0.2 // 0.0
#define F 0.0
#define IZone 100
#define STEERPOW 0.7
#define driveP .5 //0.3
#define driveI 0.0
#define driveD 0.0
#define driveF 0.15
//#define driveF 0.1 //14 tooth gear
//#define driveF 0.108 // 13 tooth gear
#define pdriveP 1.5
#define pdriveI 0.01
#define pdriveIZone 100.0
#define pdriveD 0.0
#define pdriveF 0.0
#define POTMIN 0.0
#define POTMAX 5.0
#define TOLERANCE 0.1
#define PERIOD .02
#define RATIO 1

#define FLD 1
//#define FLP 2
#define FLS 5

#define FRD 2
//#define FRP 5
#define FRS 6

#define RLD 3
//#define RLP 3
#define RLS 7

#define RRD 4
//#define RRP 4
#define RRS 8

void RobotMap::Initialize() {
	// LiveWindow* lw = LiveWindow::GetInstance();

	//serialPort = new SerialPort(9600, SerialPort::kUSB);
	//serialPort1 = new SerialPort(9600, SerialPort::kUSB1);
	//serialPort2 = new SerialPort(9600, SerialPort::kUSB2);

	imu = new AHRS(frc::I2C::kMXP); // SPI::kOnboardCS0);//serialPort, 100);
	//imu = new AHRS(SerialPort::kUSB);
	pdp = new frc::PowerDistributionPanel();

	////////////////////////////////////
	////Front Left Wheel////////////////
	////////////////////////////////////
	//Drive Motor
	driveTrainFrontLeftDrive = new WPI_TalonSRX(FLD);
	driveTrainFrontLeftDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
	driveTrainFrontLeftDrive->ConfigPeakOutputForward(1,10);
	driveTrainFrontLeftDrive->ConfigPeakOutputReverse(-1,10);
	driveTrainFrontLeftDrive->Config_kP(0,driveP,10);
	driveTrainFrontLeftDrive->Config_kI(0,driveI,10);
	driveTrainFrontLeftDrive->Config_kD(0,driveD,10);
	driveTrainFrontLeftDrive->Config_kF(0,driveF,10);
	driveTrainFrontLeftDrive->SetSensorPhase(false);
	driveTrainFrontLeftDrive->SelectProfileSlot(0,0);
	driveTrainFrontLeftDrive->Config_kP(1,pdriveP,10);
	driveTrainFrontLeftDrive->Config_kI(1,pdriveI,10);
	driveTrainFrontLeftDrive->Config_IntegralZone(1,pdriveIZone,10);
	driveTrainFrontLeftDrive->Config_kD(1,pdriveD,10);
	driveTrainFrontLeftDrive->Config_kF(1,pdriveF,10);
	//driveTrainFrontLeftDrive->ConfigMotionCruiseVelocity(3500, 10);
	//driveTrainFrontLeftDrive->ConfigMotionAcceleration(2000, 10);
	//Steering Motor
	driveTrainFrontLeftSteer = new WPI_TalonSRX(FLS);
	//driveTrainFrontLeftSteer->SetControlMode(CANTalon::kPosition);
	driveTrainFrontLeftSteer->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,10);
	driveTrainFrontLeftSteer->SelectProfileSlot(0, 0);
	driveTrainFrontLeftSteer->Config_kP(0,P,10);
	driveTrainFrontLeftSteer->Config_kI(0,I,10);
	driveTrainFrontLeftSteer->Config_kD(0,D,10);
	driveTrainFrontLeftSteer->Config_kF(0,F,10);
	driveTrainFrontLeftSteer->Config_IntegralZone(0,IZone,10);
	driveTrainFrontLeftSteer->SetSensorPhase(true);
	driveTrainFrontLeftSteer->ConfigNominalOutputForward(0,10);
	driveTrainFrontLeftSteer->ConfigNominalOutputReverse(0,10);
	driveTrainFrontLeftSteer->ConfigPeakOutputForward(STEERPOW,10);
	driveTrainFrontLeftSteer->ConfigPeakOutputReverse(-STEERPOW,10);

	////////////////////////////////////
	////Front Right Wheel///////////////
	////////////////////////////////////
	//Driving Motor
	driveTrainFrontRightDrive = new WPI_TalonSRX(FRD);
	driveTrainFrontRightDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
	driveTrainFrontRightDrive->ConfigPeakOutputForward(1,10);
	driveTrainFrontRightDrive->ConfigPeakOutputReverse(-1,10);
	driveTrainFrontRightDrive->Config_kP(0,driveP,10);
	driveTrainFrontRightDrive->Config_kI(0,driveI,10);
	driveTrainFrontRightDrive->Config_kD(0,driveD,10);
	driveTrainFrontRightDrive->Config_kF(0,driveF,10);
	driveTrainFrontRightDrive->SetSensorPhase(false);
	driveTrainFrontRightDrive->SelectProfileSlot(0,0);
	driveTrainFrontRightDrive->Config_kP(1,pdriveP,10);
	driveTrainFrontRightDrive->Config_kI(1,pdriveI,10);
	driveTrainFrontRightDrive->Config_IntegralZone(1,pdriveIZone,100);
	driveTrainFrontRightDrive->Config_kD(1,pdriveD,10);
	driveTrainFrontRightDrive->Config_kF(1,pdriveF,10);
	//driveTrainFrontRightDrive->ConfigMotionCruiseVelocity(3500, 10);
	//driveTrainFrontRightDrive->ConfigMotionAcceleration(2000, 10);
	//Steering Motor
	driveTrainFrontRightSteer = new WPI_TalonSRX(FRS);
	//driveTrainFrontRightSteer->SetControlMode(CANTalon::kPosition);
	driveTrainFrontRightSteer->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,10);
	driveTrainFrontRightSteer->SelectProfileSlot(0, 0);
	driveTrainFrontRightSteer->Config_kP(0,P,10);
	driveTrainFrontRightSteer->Config_kI(0,I,10);
	driveTrainFrontRightSteer->Config_kD(0,D,10);
	driveTrainFrontRightSteer->Config_kF(0,F,10);
	driveTrainFrontRightSteer->Config_IntegralZone(0,IZone,10);
	driveTrainFrontRightSteer->SetSensorPhase(true);
	driveTrainFrontRightSteer->ConfigNominalOutputForward(0,10);
	driveTrainFrontRightSteer->ConfigNominalOutputReverse(0,10);
	driveTrainFrontRightSteer->ConfigPeakOutputForward(STEERPOW,10);
	driveTrainFrontRightSteer->ConfigPeakOutputReverse(-STEERPOW,10);

	////////////////////////////////////
	////Rear Left Wheel/////////////////
	////////////////////////////////////
	//Driving Motor
	driveTrainRearLeftDrive = new WPI_TalonSRX(RLD);
	driveTrainRearLeftDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
	driveTrainRearLeftDrive->ConfigPeakOutputForward(1,10);
	driveTrainRearLeftDrive->ConfigPeakOutputReverse(-1,10);
	driveTrainRearLeftDrive->Config_kP(0,driveP,10);
	driveTrainRearLeftDrive->Config_kI(0,driveI,10);
	driveTrainRearLeftDrive->Config_kD(0,driveD,10);
	driveTrainRearLeftDrive->Config_kF(0,driveF+.00,10);  //  take out .05 for comp bot
	driveTrainRearLeftDrive->SetSensorPhase(false);
	driveTrainRearLeftDrive->SelectProfileSlot(0,0);
	driveTrainRearLeftDrive->Config_kP(1,pdriveP,10);
	driveTrainRearLeftDrive->Config_kI(1,pdriveI,10);
	driveTrainRearLeftDrive->Config_IntegralZone(1,pdriveIZone,10);
	driveTrainRearLeftDrive->Config_kD(1,pdriveD,10);
	driveTrainRearLeftDrive->Config_kF(1,pdriveF,10);
	//driveTrainRearLeftDrive->ConfigMotionCruiseVelocity(3500, 10);
	//driveTrainRearLeftDrive->ConfigMotionAcceleration(2000, 10);

	//Steering Motor
	driveTrainRearLeftSteer = new WPI_TalonSRX(RLS);
	//driveTrainRearLeftSteer->SetControlMode(CANTalon::kPosition);
	driveTrainRearLeftSteer->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,10);
	driveTrainRearLeftSteer->SelectProfileSlot(0, 0);
	driveTrainRearLeftSteer->Config_kP(0,P,10);
	driveTrainRearLeftSteer->Config_kI(0,I,10);
	driveTrainRearLeftSteer->Config_kD(0,D,10);
	driveTrainRearLeftSteer->Config_kF(0,F,10);
	driveTrainRearLeftSteer->Config_IntegralZone(0,IZone,10);
	driveTrainRearLeftSteer->SetSensorPhase(true);
	driveTrainRearLeftSteer->ConfigNominalOutputForward(0,10);
	driveTrainRearLeftSteer->ConfigNominalOutputReverse(0,10);
	driveTrainRearLeftSteer->ConfigPeakOutputForward(STEERPOW,10);
	driveTrainRearLeftSteer->ConfigPeakOutputReverse(-STEERPOW,10);

	////////////////////////////////////
	////Rear Right Wheel////////////////
	////////////////////////////////////
	//Driving Motor
	driveTrainRearRightDrive = new WPI_TalonSRX(RRD);
	driveTrainRearRightDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
	driveTrainRearRightDrive->ConfigPeakOutputForward(1,10);
	driveTrainRearRightDrive->ConfigPeakOutputReverse(-1,10);
	driveTrainRearRightDrive->Config_kP(0,driveP,10);
	driveTrainRearRightDrive->Config_kI(0,driveI,10);
	driveTrainRearRightDrive->Config_kD(0,driveD,10);
	driveTrainRearRightDrive->Config_kF(0,driveF,10);
	driveTrainRearRightDrive->SetSensorPhase(false);
	driveTrainRearRightDrive->SelectProfileSlot(0, 0);
	driveTrainRearRightDrive->Config_kP(1,pdriveP,10);
	driveTrainRearRightDrive->Config_kI(1,pdriveI,10);
	driveTrainRearRightDrive->Config_IntegralZone(1,pdriveIZone,10);
	driveTrainRearRightDrive->Config_kD(1,pdriveD,10);
	driveTrainRearRightDrive->Config_kF(1,pdriveF,10);
	//driveTrainRearRightDrive->ConfigMotionCruiseVelocity(3500, 10);
	//driveTrainRearRightDrive->ConfigMotionAcceleration(2000, 10);
	//Steering Motor
	driveTrainRearRightSteer = new WPI_TalonSRX(RRS);
	//driveTrainRearRightSteer->SetControlMode(CANTalon::kPosition);
	driveTrainRearRightSteer->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,10);
	driveTrainRearRightSteer->SelectProfileSlot(0,0);
	driveTrainRearRightSteer->Config_kP(0,P,10);
	driveTrainRearRightSteer->Config_kI(0,I,10);
	driveTrainRearRightSteer->Config_kD(0,D,10);
	driveTrainRearRightSteer->Config_kF(0,F,10);
	driveTrainRearRightSteer->Config_IntegralZone(0,IZone,10);
	driveTrainRearRightSteer->SetSensorPhase(true);
	driveTrainRearRightSteer->ConfigNominalOutputForward(0,10);
	driveTrainRearRightSteer->ConfigNominalOutputReverse(0,10);
	driveTrainRearRightSteer->ConfigPeakOutputForward(STEERPOW,10);
	driveTrainRearRightSteer->ConfigPeakOutputReverse(-STEERPOW,10);

	i2c = new frc::I2C(frc::I2C::Port::kMXP, 0x04);

	sonar = new frc::AnalogInput(0);

	clawMotor = new WPI_TalonSRX(13);
	clawMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,10);
	clawMotor->ConfigNominalOutputForward(0,10);
    clawMotor->ConfigNominalOutputReverse(0,10);
    clawMotor->ConfigPeakOutputForward(1,10);
    clawMotor->ConfigPeakOutputReverse(-1,10);
    clawMotor->SelectProfileSlot(0, 0);
    clawMotor->SetSensorPhase(true);
    clawMotor->Config_kP(0, 3, 10);
    clawMotor->Config_kI(0, 0, 10);
	clawMotor->Config_kD(0, 0, 10);
	clawMotor->Config_kF(0, 0.1, 10);
	clawMotor->ConfigPeakCurrentLimit(15, 10); /* 35 A */
	clawMotor->ConfigPeakCurrentDuration(1000, 10); /* 1000ms */
	clawMotor->ConfigContinuousCurrentLimit(6, 10); /* 6A */
	clawMotor->EnableCurrentLimit(true); /* turn it on */
	clawMotor->Config_kP(1, 0.1, 10);
	clawMotor->Config_kI(1, 0, 10);
	clawMotor->Config_kD(1, 0, 10);
	clawMotor->Config_kF(1, 0.06, 10);


	rollerMotor = new WPI_TalonSRX(15);//
	//rollerMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,10);
	rollerMotor->ConfigNominalOutputForward(0,10);
    rollerMotor->ConfigNominalOutputReverse(0,10);
    rollerMotor->ConfigPeakOutputForward(1,10);
    rollerMotor->ConfigPeakOutputReverse(-1,10);

	climbingMotor = new WPI_TalonSRX(14);

	armMotor = new WPI_TalonSRX(12);
	armMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,10);

	elevatorMotor = new WPI_TalonSRX(11);
	//elevatorMotor->ConfigPeakCurrentLimit(25,10);
	elevatorMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,10);
	elevatorMotor->ConfigOpenloopRamp(0, 10);
}
