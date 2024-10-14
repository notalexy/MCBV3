#include "JoystickMoveCommand.hpp"

namespace commands
{

void JoystickMoveCommand::execute()
{

        Remote* remote = &drivers->remote;
        float yawInput = 0.0f;
        float pitchInput = 0.0f;

        float h = remote->getChannel(Remote::Channel::LEFT_HORIZONTAL);
        float v = remote->getChannel(Remote::Channel::LEFT_VERTICAL);

        //TODO this lmao
        //gimbal->turretMove();

}

void JoystickMoveCommand::end(bool) {}

bool JoystickMoveCommand::isFinished(void) const { return false; }
}  // namespace commands