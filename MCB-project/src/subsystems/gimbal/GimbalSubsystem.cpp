#include "GimbalSubsystem.hpp"

int voltage;
float velocity;
namespace subsystems {

GimbalSubsystem::GimbalSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* yaw, tap::motor::DjiMotor* pitch) : tap::control::Subsystem(drivers), drivers(drivers), motorYaw(yaw), motorPitch(pitch) {
    gen = std::mt19937(rd());
    distYaw = std::uniform_int_distribution<>(-YAW_DIST_RANGE, YAW_DIST_RANGE);
    distPitch = std::uniform_int_distribution<>(-PITCH_DIST_RANGE, PITCH_DIST_RANGE);
}

void GimbalSubsystem::initialize() {
    motorPitch->initialize();
    motorYaw->initialize();
    imuOffset = getYawEncoderValue();
    targetYawAngleWorld = 0;
    targetPitchAngle = 0;
}

void GimbalSubsystem::refresh() {
    yawRadS = PI / 180 * drivers->bmi088.getGz();
    yawAngleWorld = fmod(PI / 180 * drivers->bmi088.getYaw() - imuOffset, 2 * PI);
    motorPitch->setDesiredOutput(pitchMotorVoltage);
    motorYaw->setDesiredOutput(yawMotorVoltage);
}

void GimbalSubsystem::updateMotors(float* targetYaw, float* targetPitch) {
    // impose limits on passed values
    *targetPitch = std::clamp(*targetPitch, -MAX_PITCH_DOWN, MAX_PITCH_UP);
    //*targetYaw = fmod(*targetYaw, 2 * PI);

    float yawDifference = *targetYaw - targetYawAngleWorld;
    // this deals with overshoot
    // while (yawDifference > M_PI) yawDifference -= M_TWOPI;
    // while (yawDifference < -M_PI) yawDifference += M_TWOPI;

    // set old values
    targetYawAngleWorld = *targetYaw;
    targetPitchAngle = *targetPitch;

    pitchMotorVoltage = getPitchVoltage(targetPitchAngle, dt);
    yawMotorVoltage = getYawVoltage(0, yawAngleWorld, yawRadS, targetYawAngleWorld, yawDifference / dt, dt);
}

void GimbalSubsystem::stopMotors() {
    pitchMotorVoltage = 0;
    yawMotorVoltage = 0;
    pitchController.clearBuildup();
    yawController.clearBuildup();
}

void GimbalSubsystem::reZeroYaw() {
    // TODO
}

// assume yawAngleRelativeWorld is in radians, not sure
int GimbalSubsystem::getYawVoltage(float driveTrainRPM, float yawAngleRelativeWorld, float yawRadS, float desiredAngleWorld, float inputVel, float dt) {
#if defined(yaw_sysid)
    voltage = distYaw(gen);
    velocity = yawRPM;
    return voltage;
#else
    return 1000 * yawController.calculate(yawAngleRelativeWorld, yawRadS, 0, desiredAngleWorld, inputVel, dt);
#endif
}

// assume targetangle is in radians, not sure
int GimbalSubsystem::getPitchVoltage(float targetAngle, float dt) {
#if defined(pitch_sysid)
    return distPitch(gen);
#else
    return 1000 * pitchController.calculate(getPitchEncoderValue(), getPitchVel(), targetAngle + PITCH_OFFSET, dt);
#endif
}

float GimbalSubsystem::getYawEncoderValue() { return tap::motor::DjiMotor::encoderToDegrees(motorYaw->getEncoderUnwrapped()) * PI / 180; }
float GimbalSubsystem::getPitchEncoderValue() { return tap::motor::DjiMotor::encoderToDegrees(motorPitch->getEncoderUnwrapped()) * PI / 180; }
float GimbalSubsystem::getYawVel() { return motorYaw->getShaftRPM() * PI / 30; }
float GimbalSubsystem::getPitchVel() { return motorPitch->getShaftRPM() * PI / 30; }
float GimbalSubsystem::getTargetYaw() { return targetYawAngleWorld; }
float GimbalSubsystem::getTargetPitch() { return targetPitchAngle; }
float GimbalSubsystem::getIMUOffset() { return imuOffset; }

}  // namespace subsystems
