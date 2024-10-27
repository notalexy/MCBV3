#include "GimbalSubsystem.hpp"

namespace subsystems
{
    
GimbalSubsystem::GimbalSubsystem(tap::Drivers* drivers)
   : tap::control::Subsystem(drivers),
        drivers(drivers),
          motor_Yaw(
              drivers,
              tap::motor::MotorId::MOTOR7,
              tap::can::CanBus::CAN_BUS1,
              false,
              "Yaw",
              0,
              0),
          motor_Pitch(
              drivers,
              tap::motor::MotorId::MOTOR6,
              tap::can::CanBus::CAN_BUS2,
              false,
              "Pitch",
              0,
              0)
    {
        // TODO: Complete this
    }

void GimbalSubsystem::initialize()
{
    
    motor_Pitch.initialize();
    motor_Yaw.initialize();
    // Nothing needs to be done to drivers
    // Nothing needs to be done to the controllers

    // TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's
    // reading correctly, ect)
    imuOffset = this->getYawEncoderValue();
    targetYawAngleWorld += yawAngleRelativeWorld;
}

void GimbalSubsystem::refresh()
{
    right_stick_horz =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    right_stick_vert =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    // Turning the remote raw values into values we can use more easily (circular cordinates)
    // STOP Updating stick values

    driveTrainRPM = 0;  // TODO: get this. Either power from DT motors, using yaw encoder and IMU,
                        // or something else
    yawRPM = PI / 180 * drivers->bmi088.getGz();
    yawAngleRelativeWorld = fmod(static_cast<double>(PI) / 180 * static_cast<double>(drivers->bmi088.getYaw()) - imuOffset, 2 * static_cast<double>(PI));
}

void GimbalSubsystem::turretMove(
    double desiredYawAngle,
    double desiredPitchAngle,
    double driveTrainRPM,
    double yawAngleRelativeWorld,
    double yawRPM,
    double inputVel,
    double dt)
{
    if (turretControllerTimer.execute())
    {
        pitchMotorVoltage = getPitchVoltage(desiredPitchAngle - 0.48l * static_cast<double>(PI), dt);
        yawMotorVoltage = getYawVoltage(
            driveTrainRPM,
            yawAngleRelativeWorld,
            yawRPM,
            desiredYawAngle,
            inputVel,
            dt);
    }
    // TODO: Add flywheels, indexer, and servo
}

void GimbalSubsystem::updateMotors()
{
    double temp = right_stick_horz * YAW_TURNING_PROPORTIONAL;
    driveTrainEncoder = this->getYawEncoderValue();
    yawEncoderCache = driveTrainEncoder;
    targetYawAngleWorld += temp;

    targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);
    this->turretMove(
        targetYawAngleWorld,
        0.1l * static_cast<double>(PI) * right_stick_vert,  // was - 0.5 * PI
        driveTrainRPM,
        yawAngleRelativeWorld,
        yawRPM,
        temp / dt,
        dt);
}

void GimbalSubsystem::setMotorSpeeds()
{
    motor_Pitch.setDesiredOutput(pitchMotorVoltage);
    motor_Yaw.setDesiredOutput(yawMotorVoltage);
}

void GimbalSubsystem::stopMotors()
{
    motor_Pitch.setDesiredOutput(0);
    motor_Yaw.setDesiredOutput(0);

    drivers->djiMotorTxHandler.encodeAndSendCanData();
    // TODO: Add the other motors
}

void GimbalSubsystem::reZeroYaw()
{
    // TODO
}

int GimbalSubsystem::getYawVoltage(
    double driveTrainRPM,
    double yawAngleRelativeWorld,
    double yawRPM,
    double desiredAngleWorld,
    double inputVel,
    double dt)
{
    if (robotDisabled) return 0;
    return 1000 * yawController.calculate(
                      yawAngleRelativeWorld,
                      yawRPM,
                      0,
                      desiredAngleWorld,
                      inputVel,
                      dt);  // 1000 to convert to mV which taproot wants. DTrpm is 0, can calculate
                            // and pass in the future
}

int GimbalSubsystem::getPitchVoltage(double targetAngle, double dt)
{
    if (robotDisabled) return 0;
    return 1000 * pitchController.calculate(getPitchEncoderValue(), getPitchVel(), targetAngle, dt);
}

}  // namespace ThornBots
