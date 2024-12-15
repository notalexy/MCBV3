#include "JoystickDriveCommand.hpp"

namespace commands
{

void JoystickDriveCommand::initialize() {  }
void JoystickDriveCommand::execute()
{
        x = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
        y = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        r = drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);

        translationSpeed = sqrt(x*x+y*y);
        translationAngle = x != 0 || y != 0 ? atan2(y,x) : 0;
        //TODO this lmao
        if(drivers->remote.isConnected()){
                drivetrain->moveDriveTrain(r, translationSpeed, translationAngle);
        } else {
                drivetrain->stopMotors();
        }


}

void JoystickDriveCommand::end(bool) {}

bool JoystickDriveCommand::isFinished(void) const { return false; }
}  // namespace commands