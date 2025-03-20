#include "JoystickMoveCommand.hpp"

namespace commands
{

void JoystickMoveCommand::initialize() {  }
void JoystickMoveCommand::execute()
{
        yaw = CONTROLLER_YAW_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
        pitch = CONTROLLER_PITCH_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
  
        //TODO this lmao
        if(drivers->remote.isConnected()){
                gimbal->updateMotors(&yaw, &pitch);
        } else {
                gimbal->stopMotors();
        }


}

void JoystickMoveCommand::end(bool) {}

bool JoystickMoveCommand::isFinished(void) const { return false; }
}  // namespace commands