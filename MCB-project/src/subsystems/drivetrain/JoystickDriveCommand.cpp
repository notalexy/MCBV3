#include "JoystickDriveCommand.hpp"

namespace commands
{

void JoystickDriveCommand::initialize() {  }
void JoystickDriveCommand::execute()
{
        x = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
        y = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        r = drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);

        if(drivers->remote.isConnected()){
                drivetrain->setTargetTranslation(x, y, r);
        } else {
                drivetrain->stopMotors();
        }


}

void JoystickDriveCommand::end(bool) {}

bool JoystickDriveCommand::isFinished(void) const { return false; }
}  // namespace commands