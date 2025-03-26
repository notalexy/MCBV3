#include "ShooterStopCommand.hpp"

namespace commands
{

void ShooterStopCommand::initialize() {  }
void ShooterStopCommand::execute()
{
    flywheel->setTargetVelocity(0);
}

void ShooterStopCommand::end(bool) {}

bool ShooterStopCommand::isFinished(void) const { return false; }
}  // namespace commands