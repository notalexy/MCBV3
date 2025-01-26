#include "DrivetrainSubsystem.hpp"

namespace subsystems
{

DrivetrainSubsystem::DrivetrainSubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),

      motor_one(
          drivers,
          tap::motor::MotorId::MOTOR1,
          tap::can::CanBus::CAN_BUS1,
          true,
          "Motor 1",
          0,
          0),
      motor_two(
          drivers,
          tap::motor::MotorId::MOTOR2,
          tap::can::CanBus::CAN_BUS1,
          false,
          "Motor 2",
          0,
          0),
      motor_three(
          drivers,
          tap::motor::MotorId::MOTOR3,
          tap::can::CanBus::CAN_BUS1,
          true,
          "Motor 3",
          0,
          0),
      motor_four(
          drivers,
          tap::motor::MotorId::MOTOR4,
          tap::can::CanBus::CAN_BUS1,
          false,
          "Motor 4",
          0,
          0)
{
}

void DrivetrainSubsystem::initialize()
{
    motor_one.initialize();
    motor_two.initialize();
    motor_three.initialize();
    motor_four.initialize();
}

//guaranteed to be called
void DrivetrainSubsystem::refresh()
{

    if (drivers->refSerial.getRefSerialReceivingData())  // check for uart disconnected
        powerLimit = drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;

    // shaft rpms measured from encoders
    motorOneRPM = motor_one.getShaftRPM();
    motorTwoRPM = motor_two.getShaftRPM();
    motorThreeRPM = motor_three.getShaftRPM();
    motorFourRPM = motor_four.getShaftRPM();
    motor_one.setDesiredOutput(static_cast<int32_t>(I1t * 819.2f));
    motor_two.setDesiredOutput(static_cast<int32_t>(I2t * 819.2f));
    motor_three.setDesiredOutput(static_cast<int32_t>(I3t * 819.2f));
    motor_four.setDesiredOutput(static_cast<int32_t>(I4t * 819.2f));

}
static float motorOneSpeed, motorTwoSpeed, motorThreeSpeed, motorFourSpeed = 0;

void DrivetrainSubsystem::setTargetTranslationVector(float translationSpeed, float translationAngle)
{
    //hey controller heres speed and angle
    //it will do things with the individual motors
}

void DrivetrainSubsystem::setCurrentForMotor(int motorNum, float current){
    //maybe store the four currents in an array
    //motornum would probably be 1 based, make the array size 5 and not use index 0
    //currents[motorNum]=current
}

//fix function
void DrivetrainSubsystem::stopMotors()
{
    motor_one.setDesiredOutput(0);
    motor_two.setDesiredOutput(0);
    motor_three.setDesiredOutput(0);
    motor_four.setDesiredOutput(0);
}


}  // namespace subsystems