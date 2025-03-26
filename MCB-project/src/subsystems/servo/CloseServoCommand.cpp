#include "CloseServoCommand.hpp"

namespace commands
{

void OpenServoCommand::initialize() {}

void OpenServoCommand::execute()
{
    servo->setTargetPosition(0.0f);
}

void OpenServoCommand::end(bool) {}

bool OpenServoCommand::isFinished(void) const {
    return servo->movementComplete();
}
}  // namespace commands