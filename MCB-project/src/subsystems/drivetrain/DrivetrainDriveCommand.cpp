#include "DrivetrainDriveCommand.hpp"

#include "util/Pose2d.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"

namespace commands {

void DrivetrainDriveCommand::initialize() {}
void DrivetrainDriveCommand::execute() {
    float referenceAngle = gimbal->getYawEncoderValue();

    // if (controlMode == ControlMode::KEYBOARD) {
    //     x = drivers->remote.keyPressed(Remote::Key::A) ? -1 : (drivers->remote.keyPressed(Remote::Key::D) ? 1 : 0);
    //     y = drivers->remote.keyPressed(Remote::Key::W) ? 1 : (drivers->remote.keyPressed(Remote::Key::S) ? -1 : 0);

    // } else if (controlMode == ControlMode::CONTROLLER) {
    x = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
    y = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
    // } else {
    //     drivetrain->stopMotors();
    //     return;
    // }

    if (driveMode == DriveMode::BEYBLADE) {
        r = 0.4;
    } else if (driveMode == DriveMode::BEYBLADE2) {
        r = 0.8;
    } else {
        float targetAngle = 0.0f;
        if (driveMode == DriveMode::PEEK_LEFT) {
            targetAngle = subsystems::PEEK_LEFT_AMT;
        } else if (driveMode == DriveMode::PEEK_RIGHT) {
            targetAngle = subsystems::PEEK_RIGHT_AMT;
        }
        r = drivetrain->calculateRotationPID(targetAngle + referenceAngle);
    }

    Pose2d drive(x, y, r);

    drivetrain->setTargetTranslation(drive.rotate(referenceAngle));
}

bool DrivetrainDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

void DrivetrainDriveCommand::end(bool cancel) {}

}  // namespace commands