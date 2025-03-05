#include "GimbalSubsystem.hpp"

int voltage;
float velocity;
namespace subsystems
{

GimbalSubsystem::GimbalSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* yaw, tap::motor::DjiMotor* pitch) : tap::control::Subsystem(drivers), drivers(drivers), motorYaw(yaw), motorPitch(pitch)
{
    gen = std::mt19937(rd());
    distYaw = std::uniform_int_distribution<>(-YAW_DIST_RANGE, YAW_DIST_RANGE);
    distPitch = std::uniform_int_distribution<>(-PITCH_DIST_RANGE, PITCH_DIST_RANGE);
}

void GimbalSubsystem::initialize()
{
    motorPitch->initialize();
    motorYaw->initialize();
    imuOffset = getYawEncoderValue();
    targetYawAngleWorld += yawAngleRelativeWorld;
}

void GimbalSubsystem::refresh()
{
    driveTrainRPM = 0;
    yawRPM = PI / 180 * drivers->bmi088.getGz();
    yawAngleRelativeWorld = fmod(PI / 180 * drivers->bmi088.getYaw() - imuOffset, 2 * PI);
    motorPitch->setDesiredOutput(pitchMotorVoltage);
    motorYaw->setDesiredOutput(yawMotorVoltage);
}

void GimbalSubsystem::updateMotors(float* changeInTargetYaw, float* targetPitch)
{
    *targetPitch = std::clamp(*targetPitch, -MAX_PITCH_DOWN, MAX_PITCH_UP);
    driveTrainEncoder = getYawEncoderValue();
    yawEncoderCache = driveTrainEncoder;
    targetYawAngleWorld = fmod(targetYawAngleWorld + *changeInTargetYaw, 2 * PI);
    pitchMotorVoltage = getPitchVoltage(*targetPitch, dt);
    yawMotorVoltage = getYawVoltage(driveTrainRPM, yawAngleRelativeWorld, yawRPM, targetYawAngleWorld, *changeInTargetYaw / dt, dt);
}

void GimbalSubsystem::stopMotors()
{
    pitchMotorVoltage = 0;
    yawMotorVoltage = 0;
    pitchController.clearBuildup();
    yawController.clearBuildup();
}

void GimbalSubsystem::reZeroYaw()
{
    // TODO
}

// assume yawAngleRelativeWorld is in radians, not sure
int GimbalSubsystem::getYawVoltage(float driveTrainRPM, float yawAngleRelativeWorld, float yawRPM, float desiredAngleWorld, float inputVel, float dt)
{
#if defined(yaw_sysid)
    voltage = distYaw(gen);
    velocity = yawRPM;
    return voltage;
#else
    return 1000 * yawController.calculate(yawAngleRelativeWorld, yawRPM, 0, desiredAngleWorld, inputVel, dt);
#endif
}

// assume targetangle is in radians, not sure
int GimbalSubsystem::getPitchVoltage(float targetAngle, float dt)
{
#if defined(pitch_sysid)
    return distPitch(gen);
#else
    return 1000 * pitchController.calculate(getPitchEncoderValue(), getPitchVel(), targetAngle + PITCH_OFFSET, dt);
#endif
}

float GimbalSubsystem::getYawEncoderValue() { return tap::motor::DjiMotor::encoderToDegrees(motorYaw->getEncoderUnwrapped()) * PI / 180 - YAW_OFFSET; }
float GimbalSubsystem::getPitchEncoderValue() { return tap::motor::DjiMotor::encoderToDegrees(motorPitch->getEncoderUnwrapped()) * PI / 180; }
float GimbalSubsystem::getYawVel() { return motorYaw->getShaftRPM() * PI / 30; }
float GimbalSubsystem::getPitchVel() { return motorPitch->getShaftRPM() * PI / 30; }

}  // namespace subsystems
