#include "JoystickMoveCommand.h"

namespace commands
{

void JoystickMoveCommand::initialize() {  }
void JoystickMoveCommand::execute()
{
        yaw = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
        pitch = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
  
        //TODO this lmao
        if(drivers->remote.isConnected()){
                gimbal->updateMotors(yaw, pitch);
        } else {
                gimbal->stopMotors();
        }


}

void JoystickMoveCommand::end(bool) {}

bool JoystickMoveCommand::isFinished(void) const { return false; }
}  // namespace commands