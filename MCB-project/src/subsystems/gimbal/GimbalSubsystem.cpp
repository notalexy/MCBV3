#include "GimbalSubsystem.hpp"

namespace subsystems {

GimbalSubsystem::GimbalSubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      motor_Yaw(drivers, tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS1, false, "Yaw", 0, 0),
      motor_Pitch(drivers, tap::motor::MotorId::MOTOR6, tap::can::CanBus::CAN_BUS2, false, "Pitch", 0, 0) {}

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
}

void GimbalSubsystem::turretMove(float desiredYawAngle, float desiredPitchAngle, float driveTrainRPM,
                                 float yawAngleRelativeWorld, float yawRPM, float inputVel, float dt) {
 //   if (turretControllerTimer.execute()) {
        pitchMotorVoltage = getPitchVoltage(desiredPitchAngle - 0.48 * PI, dt);
        yawMotorVoltage = getYawVoltage(driveTrainRPM, yawAngleRelativeWorld, yawRPM, desiredYawAngle, inputVel, dt);
   // }
}

void GimbalSubsystem::updateMotors(float right_stick_horz, float right_stick_vert) {
    float temp = right_stick_horz * YAW_TURNING_PROPORTIONAL;
    driveTrainEncoder = getYawEncoderValue();
    yawEncoderCache = driveTrainEncoder;
    targetYawAngleWorld = fmod(targetYawAngleWorld + temp, 2 * PI);
    turretMove(targetYawAngleWorld, 0.1 * PI * right_stick_vert, driveTrainRPM, yawAngleRelativeWorld, yawRPM, temp / dt, dt);
    setMotorSpeeds();
}

void GimbalSubsystem::setMotorSpeeds() {
    motor_Pitch.setDesiredOutput(pitchMotorVoltage);
    motor_Yaw.setDesiredOutput(yawMotorVoltage);
}

void GimbalSubsystem::stopMotors() {
    motor_Pitch.setDesiredOutput(0);
    motor_Yaw.setDesiredOutput(0);
    drivers->djiMotorTxHandler.encodeAndSendCanData();
}

void GimbalSubsystem::reZeroYaw() {
    // TODO
}

int GimbalSubsystem::getYawVoltage(float driveTrainRPM, float yawAngleRelativeWorld, float yawRPM,
                                   float desiredAngleWorld, float inputVel, float dt) {
    return robotDisabled ? 0 : 1000 * yawController.calculate(yawAngleRelativeWorld, yawRPM, 0, desiredAngleWorld, inputVel, dt);
}

int GimbalSubsystem::getPitchVoltage(float targetAngle, float dt) {
    return robotDisabled ? 0 : 1000 * pitchController.calculate(getPitchEncoderValue(), getPitchVel(), targetAngle, dt);
}

}  // namespace ThornBots
