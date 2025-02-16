#include "IndexerSubsystem.hpp"

namespace subsystems {
    
IndexerSubsystem::IndexerSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* index)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    motor_Indexer(index)
    {}

void IndexerSubsystem::initialize() {
    motor_Indexer->initialize();
}

void IndexerSubsystem::refresh() {
    motor_Indexer->setDesiredOutput(indexerVoltage);
}

void IndexerSubsystem::indexAtRate(float ballsPerSecond) {
    this->ballsPerSecond = ballsPerSecond;
    setTargetMotorRPM(ballsPerSecond * 60.0f * REV_PER_BALL);
}

void IndexerSubsystem::stopIndex() {
    indexerVoltage = 0;
}


//first 1800 degrees until first shot
//need to tell it to go max speed until we get to 1800 degrees from where we started
//so the first shot goes out asap

void IndexerSubsystem::setTargetMotorRPM(int targetMotorRPM) {
    indexPIDController.runControllerDerivateError(targetMotorRPM - motor_Indexer->getShaftRPM(), 1);

    indexerVoltage = static_cast<int32_t>(indexPIDController.getOutput());
}

// converts delta motor ticks to num balls shot using constants
float IndexerSubsystem::getNumBallsShot() {
    return (motor_Indexer->getEncoderUnwrapped() - numTicksAtInit) / tap::motor::DjiMotor::ENC_RESOLUTION / REV_PER_BALL;
}

void IndexerSubsystem::resetBallsCounter() {
    numTicksAtInit = motor_Indexer->getEncoderUnwrapped();
}

float IndexerSubsystem::getBallsPerSecond() {
    return ballsPerSecond;
}

} //namespace subsystems