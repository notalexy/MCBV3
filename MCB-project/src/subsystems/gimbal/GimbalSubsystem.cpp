#include "GimbalSubsystem.hpp"

int voltage; 
float velocity;
namespace subsystems {

GimbalSubsystem::GimbalSubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      motor_Yaw(drivers, tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS1, false, "Yaw", 0, 0),
      motor_Pitch(drivers, tap::motor::MotorId::MOTOR6, tap::can::CanBus::CAN_BUS2, false, "Pitch", 0, 0) {
        gen = std::mt19937(rd());
        distYaw = std::uniform_int_distribution<>(-YAW_DIST_RANGE, YAW_DIST_RANGE);
        distPitch = std::uniform_int_distribution<>(-PITCH_DIST_RANGE, PITCH_DIST_RANGE);
      }

void GimbalSubsystem::initialize() {
    motor_Pitch.initialize();
    motor_Yaw.initialize();
    imuOffset = getYawEncoderValue();
    targetYawAngleWorld += yawAngleRelativeWorld;
}

void GimbalSubsystem::refresh() {
    driveTrainRPM = 0;
    yawRPM = PI / 180 * drivers->bmi088.getGz();
    yawAngleRelativeWorld = fmod(PI / 180 * drivers->bmi088.getYaw() - imuOffset, 2 * PI);
    motor_Pitch.setDesiredOutput(pitchMotorVoltage);
    motor_Yaw.setDesiredOutput(yawMotorVoltage);
}


void GimbalSubsystem::updateMotors(float* changeInTargetYaw, float* targetPitch) {
    *targetPitch = std::clamp(*targetPitch, -MAX_PITCH_DOWN, MAX_PITCH_UP);
    driveTrainEncoder = getYawEncoderValue();
    yawEncoderCache = driveTrainEncoder;
    targetYawAngleWorld = fmod(targetYawAngleWorld + *changeInTargetYaw, 2 * PI);
    pitchMotorVoltage = getPitchVoltage(*targetPitch, dt);
    yawMotorVoltage = getYawVoltage(driveTrainRPM, yawAngleRelativeWorld, yawRPM, targetYawAngleWorld, *changeInTargetYaw / dt, dt);
    
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

//assume yawAngleRelativeWorld is in radians, not sure
int GimbalSubsystem::getYawVoltage(float driveTrainRPM, float yawAngleRelativeWorld, float yawRPM,
                                   float desiredAngleWorld, float inputVel, float dt) {
    #if defined(yaw_sysid)
    voltage = distYaw(gen);
    velocity = yawRPM;
    return voltage;
    #else
    return 1000 * yawController.calculate(yawAngleRelativeWorld, yawRPM, 0, desiredAngleWorld, inputVel, dt);
    #endif
}

//assume targetangle is in radians, not sure
int GimbalSubsystem::getPitchVoltage(float targetAngle, float dt) {
    #if defined(pitch_sysid)
    return distPitch(gen);
    #else
    return 1000 * pitchController.calculate(getPitchEncoderValue(), getPitchVel(), targetAngle + PITCH_OFFSET, dt);
    #endif
}


}  // namespace ThornBots
