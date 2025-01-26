#include "IndexerSubsystem.hpp"

namespace subsystems {
    
IndexerSubsystem::IndexerSubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers)
      {}

void IndexerSubsystem::initialize() {
  motor_Indexer.initialize();
}

void IndexerSubsystem::refresh() {
  motor_Indexer.setDesiredOutput(indexerVoltage);
}

void IndexerSubsystem::indexAtRate(float ballsPerSecond) {
  this->ballsPerSecond = ballsPerSecond;
  setTargetMotorRPM(ballsPerSecond * 60.0f * REV_PER_BALL);
}

void IndexerSubsystem::setTargetMotorRPM(int targetMotorRPM) {
  indexPIDController.runControllerDerivateError(targetMotorRPM - motor_Indexer.getShaftRPM(), 1);

  indexerVoltage = static_cast<int32_t>(indexPIDController.getOutput());
}

float IndexerSubsystem::getNumBallsShot() {
  return (motor_Indexer.getEncoderUnwrapped() - numTicksAtInit) / tap::motor::DjiMotor::ENC_RESOLUTION / REV_PER_BALL;
}

void IndexerSubsystem::resetBallsCounter() {
  numTicksAtInit = motor_Indexer.getEncoderUnwrapped();
}

} //namespace subsystems