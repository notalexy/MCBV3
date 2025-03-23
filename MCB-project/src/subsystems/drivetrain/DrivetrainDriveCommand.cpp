#include "DrivetrainDriveCommand.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"

namespace commands {

void DrivetrainDriveCommand::execute() {
    float referenceAngle = gimbal->getYawEncoderValue();


    if (controlMode == ControlMode::KEYBOARD) {
        x = drivers->remote.keyPressed(Remote::Key::A) ? -1 : (drivers->remote.keyPressed(Remote::Key::D) ? 1 : 0);
        y = drivers->remote.keyPressed(Remote::Key::W) ? 1 : (drivers->remote.keyPressed(Remote::Key::S) ? -1 : 0);

    } else if (controlMode == ControlMode::CONTROLLER) {
        x = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        y = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
    }

    if (driveMode == DriveMode::BEYBLADE) {
        r = 0.5;
    } else {
        float targetAngle = 0.0f;
        if (driveMode == DriveMode::PEEK_LEFT) {
            targetAngle = subsystems::PEEK_LEFT_AMT;
        } else if (driveMode == DriveMode::PEEK_RIGHT) {
            targetAngle = subsystems::PEEK_RIGHT_AMT;
        }
        r = drivetrain->calculateRotationPID(r - referenceAngle);
    }


    Pose2d drive(x, y, r);

    drivetrain->setTargetTranslation(drive.rotate(referenceAngle));
}

}  // namespace commands