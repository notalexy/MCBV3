#include "StopCommand.h"

namespace commands
{

void StopCommand::initialize() {  }
void StopCommand::execute()
{
    flywheel->stopMotors();
}

void StopCommand::end(bool) {}

bool StopCommand::isFinished(void) const { return false; }
}  // namespace commands