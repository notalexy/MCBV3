#include "IndexerSubsystem2Motor.hpp"

namespace subsystems
{

IndexerWithSecondMotorSubsystem::IndexerWithSecondMotorSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2)
    : IndexerSubsystem(drivers, index1), // Call base class constructor
    motor_Indexer2(index2)
{

    // Any additional initialization for the second motor, if necessary
}

void IndexerWithSecondMotorSubsystem::initialize() {
    // Initialize both motors
    motor_Indexer2->initialize();     // Initialize the second motor
}

void IndexerWithSecondMotorSubsystem::refresh() {
    // Set the desired output for both motors
    motor_Indexer2->setDesiredOutput(indexerVoltage2);   // Second motor (same voltage)
}

void IndexerWithSecondMotorSubsystem::stopIndex() {
    // Stop both motors
    IndexerSubsystem::stopIndex();
    indexerVoltage2 = 0;
}

void IndexerWithSecondMotorSubsystem::indexAtRate(float ballsPerSecond){
    this->ballsPerSecond = ballsPerSecond;
    //divided by 2
    IndexerSubsystem::setTargetMotorRPM(ballsPerSecond * 30.0f * REV_PER_BALL);
    setTargetMotorRPM(ballsPerSecond * 30.0f * REV_PER_BALL);
}
void IndexerWithSecondMotorSubsystem::setTargetMotorRPM(int targetMotorRPM){

    indexPIDController2.runControllerDerivateError(targetMotorRPM - motor_Indexer2->getShaftRPM(), 1);

    indexerVoltage2 = static_cast<int32_t>(indexPIDController2.getOutput());
}

} // namespace subsystems
