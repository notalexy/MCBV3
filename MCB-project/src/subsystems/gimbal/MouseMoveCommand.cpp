#include "MouseMoveCommand.hpp"

namespace commands
{

void MouseMoveCommand::initialize() { 
        mouseXOffset = drivers->remote.getMouseX();
        mouseYOffset = drivers->remote.getMouseY();
        
 }
void MouseMoveCommand::execute()
{


        yaw = MOUSE_YAW_PROPORTIONAL * (drivers->remote.getMouseX() - mouseXOffset);
        pitch += MOUSE_PITCH_PROPORTIONAL * (drivers->remote.getMouseY() - mouseYOffset);

        //TODO this lmao
        if(drivers->remote.isConnected()){
                gimbal->updateMotors(&yaw, &pitch);
        } else {
                gimbal->stopMotors();
        }


}

void MouseMoveCommand::end(bool) {
        pitch = 0;
}

bool MouseMoveCommand::isFinished() const { return false; }
}  // namespace commands