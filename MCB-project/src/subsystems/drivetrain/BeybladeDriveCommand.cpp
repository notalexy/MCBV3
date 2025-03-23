#include "BeybladeDriveCommand.hpp"

namespace commands
{


Pose2d JoystickDriveCommand::getDrive()
{
        float referenceAngle = gimbal->getYawEncoderValue();

        x = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        y = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);

        if (mode == DriveMode::BEYBLADE) {
            r = 0.5;
        } else if (mode == DriveMode::FOLLOW_TURRET) {
            r = gimbal->getYawEncoderValue() - referenceAngle;
        }
        r = drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN ? 0.5 : 0;

        return Pose2d(x, y, r).rotate(referenceAngle);
}

}  // namespace commands