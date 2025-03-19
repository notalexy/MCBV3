#include "DrivetrainFollowTurretCommand.hpp"

namespace commands
{

void DrivetrainFollowTurretCommand::initialize() {}
void DrivetrainFollowTurretCommand::execute()
{
        float referenceAngle = gimbal->getYawEncoderValue();

        x = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        y = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
        float angleError = angleOffset - referenceAngle;

        if (angleError > M_PI)
        {
                angleError -= 2 * M_PI;
        }
        else if (angleError < -M_PI)
        {
                angleError += 2 * M_PI;
        }

        r = angleError * 2;

        Pose2d drive(x, y, r);


        if(drivers->remote.isConnected()){
                drivetrain->setTargetTranslation(drive.rotate(referenceAngle));
        } else {
                drivetrain->stopMotors();
        }


}

void DrivetrainFollowTurretCommand::end(bool) {}

bool DrivetrainFollowTurretCommand::isFinished(void) const { return false; }
}  // namespace commands