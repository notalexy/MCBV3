#include "MouseMoveCommand.h"

namespace commands
{

void MouseMoveCommand::initialize() {  }
void MouseMoveCommand::execute()
{

        static int mouseXOffset = drivers->remote.getMouseX();
        static int mouseYOffset = drivers->remote.getMouseY();
        int mouseX = drivers->remote.getMouseX() - mouseXOffset;
        int mouseY = drivers->remote.getMouseY() - mouseYOffset;
        static double pitch=0;

        //need to tune the numbers here, then pull them out as constants
        pitch+=mouseY / 10000.0;
        yaw = mouseX  / 200.0;
  
        //TODO this lmao
        if(drivers->remote.isConnected()){
                gimbal->updateMotors(yaw, pitch);
        } else {
                gimbal->stopMotors();
        }


}

void MouseMoveCommand::end(bool) {}

bool MouseMoveCommand::isFinished(void) const { return false; }
}  // namespace commands