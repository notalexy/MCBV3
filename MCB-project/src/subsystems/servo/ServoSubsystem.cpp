#include "ServoSubsystem.hpp"

namespace subsystems {
    
ServoSubsystem::ServoSubsystem(tap::Drivers* drivers, tap::motor::Servo* servo)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    servo(servo)
    {}

void ServoSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);
}

void ServoSubsystem::refresh() {
    servo->updateSendPwmRamp();
}

void ServoSubsystem::setTargetPosition(float position) {
    servo->setTargetPwm(position);
}

bool ServoSubsystem::movementComplete() {
    return servo->isRampTargetMet();
}

} //namespace subsystems