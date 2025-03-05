#include "GimbalLockCommand.hpp"

namespace commands
{

        

void GimbalLockCommand::initialize() {  }
void GimbalLockCommand::execute()
{
        yaw = 0;
        pitch = CONTROLLER_PITCH_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
  
        //TODO this lmao
        if(drivers->remote.isConnected()){
                gimbal->updateMotors(&yaw, &pitch);
        } else {
                gimbal->stopMotors();
        }
}

void GimbalLockCommand::end(bool) {}

bool GimbalLockCommand::isFinished(void) const { return false; }
}  // namespace commands