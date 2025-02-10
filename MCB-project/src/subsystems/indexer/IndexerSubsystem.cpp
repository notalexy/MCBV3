#include "IndexerSubsystem.hpp"

#if defined(sentry)
motor = tap::motor::MotorId::MOTOR5
#else
using tap::motor::MotorId::MOTOR8 motor
#endif
namespace subsystems {
    
IndexerSubsystem::IndexerSubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    #if defined(sentry)  
    motor_Indexer(tap::motor::DjiMotor(drivers, motor, tap::can::CanBus::CAN_BUS2, false, "Indexer2", 0, 0))
    #else
    motor_Indexer(tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS2, false, "Indexer2", 0, 0))
    #endif
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

void IndexerSubsystem::stopIndex() {
    indexerVoltage = 0;
}


//first 1800 degrees until first shot
//need to tell it to go max speed until we get to 1800 degrees from where we started
//so the first shot goes out asap

void IndexerSubsystem::setTargetMotorRPM(int targetMotorRPM) {
    indexPIDController.runControllerDerivateError(targetMotorRPM - motor_Indexer.getShaftRPM(), 1);

    indexerVoltage = static_cast<int32_t>(indexPIDController.getOutput());
}

// converts delta motor ticks to num balls shot using constants
float IndexerSubsystem::getNumBallsShot() {
    return (motor_Indexer.getEncoderUnwrapped() - numTicksAtInit) / tap::motor::DjiMotor::ENC_RESOLUTION / REV_PER_BALL;
}

void IndexerSubsystem::resetBallsCounter() {
    numTicksAtInit = motor_Indexer.getEncoderUnwrapped();
}

float IndexerSubsystem::getBallsPerSecond() {
    return ballsPerSecond;
}

} //namespace subsystems