#include "Subsystems/Claw.h"
#include "RobotMap.h"
#include "ctre/phoenix.h"
#include "Robot.h"
#include "OI.h"

constexpr float MAX_OPEN = -2700;
constexpr float CURRENT = 5;

Claw::Claw() : Subsystem("Claw") {
	clawMotor = RobotMap::clawMotor;
}

void Claw::InitDefaultCommand() {

}

void Claw::clawClamp(float speed) {
	clawMotor->SelectProfileSlot(1, 0);
   if(Robot::oi->GetLeftBumper() == 1){
		clawMotor->Set(ControlMode::Current,-CURRENT * 2);
	}else{
		clawMotor->Set(ControlMode::Current, -CURRENT);
	}
}

void Claw::clawUnclamp() {
	/*clawMotor->SelectProfileSlot(0, 0);
	int count = 0;
	int clawCount = clawMotor->GetSelectedSensorPosition(0);

	while(clawCount < -4096){
		count ++;
		clawCount += 4096;
	}

	while(clawCount >= 0){
		count--;
		clawCount -= 4096;
	}

	clawMotor->Set(ControlMode::Position, MAX_OPEN- (4096*count));*/
	clawMotor->SelectProfileSlot(1, 0);
	clawMotor->Set(ControlMode::Current, 4);
}

void Claw::clawStop() {
	clawMotor->SelectProfileSlot(0, 0);
	clawMotor->Set(ControlMode::PercentOutput,0);
}

float Claw::getClawPosition(){
	return clawMotor->GetSelectedSensorPosition(0);
}
