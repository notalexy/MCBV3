#include "StartCommand.h"

namespace commands
{

void StartCommand::initialize() {  }
void StartCommand::execute()
{
    flywheel->setTargetVelocity(flywheel->FLYWHEEL_MOTOR_MAX_RPM);
}

void StartCommand::end(bool) {}

// doesn't have a real end condition
bool StartCommand::isFinished(void) const { return false; }
}  // namespace commands