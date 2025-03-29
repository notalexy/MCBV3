#include "DrivetrainDriveCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {

void DrivetrainDriveCommand::initialize() {
    x = 0;
    y = 0;
    r = 0;
}
void DrivetrainDriveCommand::execute() {
    float referenceAngle = gimbal->getYawEncoderValue();

    if (controlMode == ControlMode::KEYBOARD) {
        x = (drivers->remote.keyPressed(Remote::Key::D) - drivers->remote.keyPressed(Remote::Key::A)) * 1.5;
        y = (drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S)) * 1.5;

    } else if (controlMode == ControlMode::CONTROLLER) {
        x = 1.75 * drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        y = 1.75 * drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
    } else {
        drivetrain->stopMotors();
        return;
    }

    if (driveMode == DriveMode::BEYBLADE) {
        r = 0.4;
    } else if (driveMode == DriveMode::BEYBLADE2) {
        r = 0.8;
    } else if (driveMode == DriveMode::NO_SPIN) {
        r = 0;

    } else {
        float targetAngle = 0.0f;
        if (driveMode == DriveMode::PEEK_LEFT) {
            targetAngle = PEEK_LEFT_AMT;
        } else if (driveMode == DriveMode::PEEK_RIGHT) {
            targetAngle = PEEK_RIGHT_AMT;
        }
        r = drivetrain->calculateRotationPID(targetAngle + referenceAngle);  // + M_PI));
    }

    Pose2d drive(x, y, r);

    drivetrain->setTargetTranslation(drive.rotate(referenceAngle));
}

bool DrivetrainDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

void DrivetrainDriveCommand::end(bool cancel) {}

}  // namespace commands