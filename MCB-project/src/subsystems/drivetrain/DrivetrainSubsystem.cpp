#include "DrivetrainSubsystem.hpp"

namespace subsystems
{

DrivetrainSubsystem::DrivetrainSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* motorOne, tap::motor::DjiMotor* motorTwo, tap::motor::DjiMotor* motorThree, tap::motor::DjiMotor* motorFour)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      motorOne(motorOne),
      motorTwo(motorOne),
      motorThree(motorThree),
      motorFour(motorFour)
      
{
}

void DrivetrainSubsystem::initialize()
{
    motorOne->initialize();
    motorTwo->initialize();
    motorThree->initialize();
    motorFour->initialize();
}

//guaranteed to be called
void DrivetrainSubsystem::refresh()
{

    if (drivers->refSerial.getRefSerialReceivingData())  // check for uart disconnected
        powerLimit = drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;

    // shaft rpms measured from encoders
    motorOneRPM = motorOne->getShaftRPM();
    motorTwoRPM = motorTwo->getShaftRPM();
    motorThreeRPM = motorThree->getShaftRPM();
    motorFourRPM = motorFour->getShaftRPM();
    motorOne->setDesiredOutput(static_cast<int32_t>(I1t * 819.2f));
    motorTwo->setDesiredOutput(static_cast<int32_t>(I2t * 819.2f));
    motorThree->setDesiredOutput(static_cast<int32_t>(I3t * 819.2f));
    motorFour->setDesiredOutput(static_cast<int32_t>(I4t * 819.2f));

}
static float motorOneSpeed, motorTwoSpeed, motorThreeSpeed, motorFourSpeed = 0;

void DrivetrainSubsystem::setTargetTranslationVector(float translationSpeed, float translationAngle)
{
    //hey controller heres speed and angle
    //it will do things with the individual motors
}

// void DrivetrainSubsystem::setCurrentForMotor(int motorNum, float current){
//     //maybe store the four currents in an array
//     //motornum would probably be 1 based, make the array size 5 and not use index 0
//     //currents[motorNum]=current
// }

//fix function
void DrivetrainSubsystem::stopMotors()
{
    motorOne->setDesiredOutput(0);
    motorTwo->setDesiredOutput(0);
    motorThree->setDesiredOutput(0);
    motorFour->setDesiredOutput(0);
}


}  // namespace subsystems