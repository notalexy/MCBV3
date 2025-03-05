#include "DrivetrainSubsystem.hpp"

namespace subsystems
{

DrivetrainSubsystem::DrivetrainSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* motorOne, tap::motor::DjiMotor* motorTwo, tap::motor::DjiMotor* motorThree, tap::motor::DjiMotor* motorFour)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      motorArray{motorOne, motorTwo, motorThree, motorFour}
{
}

void DrivetrainSubsystem::initialize()
{
    for(tap::motor::DjiMotor* m : motorArray) m->initialize();
}

//guaranteed to be called
void DrivetrainSubsystem::refresh()
{   
    //need to actually fix this yay
    if (drivers->refSerial.getRefSerialReceivingData())  // check for uart disconnected
        powerLimit = drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;

    for(int i = 0; i < 4; i ++){
        float adjustedCurrent = std::clamp(motorCurrent[i], -20.0f, 20.0f) * 819.2f;
        motorArray[i]->setDesiredOutput(static_cast<int32_t>(adjustedCurrent));

        motorVel[i] = motorArray[i]->getShaftRPM() * PI / 30.0f; //in rad/s
    }

}

void DrivetrainSubsystem::setTargetTranslation(float x, float y, float rot, float angleReference)
{       
    lastDrive = Pose2d(x, y, rot);

    controller.calculate(lastDrive, angleReference, motorVel, motorCurrent);
    
    // motorCurrent[0] = 1.0f;
    // motorCurrent[0] = 10.0f;
}


//fix function
void DrivetrainSubsystem::stopMotors()
{
    for(int i = 0; i < 4; i++) motorCurrent[i] = 0;
}


}  // namespace subsystems