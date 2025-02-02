#include "ShooterStopCommand.h"

namespace commands
{

void ShooterStopCommand::initialize() {  }
void ShooterStopCommand::execute()
{
    flywheel->stopMotors();
}

void ShooterStopCommand::end(bool) {}

bool ShooterStopCommand::isFinished(void) const { return false; }
}  // namespace commands