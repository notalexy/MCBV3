#include "JoystickDriveCommand.hpp"

namespace commands
{

void JoystickDriveCommand::initialize() {  }
void JoystickDriveCommand::execute()
{
        float referenceAngle = (drivers->bmi088.getYaw()-180) * PI/180 - gimbal->getYawEncoderValue();

        x = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        y = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
        r = drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN ? 0.5 : 0;

        if(drivers->remote.isConnected()){
                drivetrain->setTargetTranslation(x, y, r, referenceAngle);
        } else {
                drivetrain->stopMotors();
        }


}

void JoystickDriveCommand::end(bool) {}

bool JoystickDriveCommand::isFinished(void) const { return false; }
}  // namespace commands