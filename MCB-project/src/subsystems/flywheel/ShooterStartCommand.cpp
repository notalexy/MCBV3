#include "ShooterStartCommand.hpp"
#include "subsystems/flywheel/FlywheelSubsystemConstants.hpp"

namespace commands
{

void ShooterStartCommand::initialize() {  }
void ShooterStartCommand::execute()
{
    flywheel->setTargetVelocity(subsystems::FLYWHEEL_MOTOR_MAX_RPM);
}

void ShooterStartCommand::end(bool) {}

// doesn't have a real end condition
bool ShooterStartCommand::isFinished(void) const { return !drivers->remote.isConnected(); }
}  // namespace commands