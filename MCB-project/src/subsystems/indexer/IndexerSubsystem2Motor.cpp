#include "IndexerSubsystem2Motor.hpp"

namespace subsystems
{

IndexerWithSecondMotorSubsystem::IndexerWithSecondMotorSubsystem(tap::Drivers* drivers)
    : IndexerSubsystem(drivers), // Call base class constructor
    motor_Indexer2(tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS2, false, "Indexer2", 0, 0)),
    motor_Indexer(tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS2, false, "Indexer2", 0, 0))

{

    // Any additional initialization for the second motor, if necessary
}

void IndexerWithSecondMotorSubsystem::initialize() {
    // Initialize both motors
    motor_Indexer.initialize();  // Initialize the first motor
    motor_Indexer2.initialize();     // Initialize the second motor
}

void IndexerWithSecondMotorSubsystem::refresh() {
    // Set the desired output for both motors
    motor_Indexer.setDesiredOutput(indexerVoltage);    // First motor
    motor_Indexer2.setDesiredOutput(indexerVoltage);   // Second motor (same voltage)
}

void IndexerWithSecondMotorSubsystem::stopIndex() {
    // Stop both motors
    motor_Indexer.setDesiredOutput(0);
    motor_Indexer2.setDesiredOutput(0);
}

} // namespace subsystems
