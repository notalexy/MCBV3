#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"

#include "drivers.hpp"

namespace commands {
using subsystems::DrivetrainSubsystem;
using subsystems::GimbalSubsystem;

using tap::communication::serial::Remote;

enum DriveMode { BEYBLADE, BEYBLADE2, NO_SPIN, FOLLOW_TURRET, PEEK_LEFT, PEEK_RIGHT };
enum ControlMode { KEYBOARD, CONTROLLER, DISABLED  };

class DrivetrainDriveCommand : public tap::control::Command {
public:
    DrivetrainDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, DriveMode driveMode, ControlMode controlMode)
        : drivers(drivers),
          drivetrain(drive),
          gimbal(gimbal),
          driveMode(driveMode),
          controlMode(controlMode) {
        addSubsystemRequirement(drive);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

<<<<<<< HEAD
    bool isFinished() const { return false; };
=======
    bool isFinished() const override;
>>>>>>> fdb5fb34b678057dd1df5f762472988d8c535be9

    const char* getName() const override { return "drive command"; }

    void setDriveMode(DriveMode newDriveMode) { driveMode = newDriveMode; } 

    void setControlMode(ControlMode newControlMode) { controlMode = newControlMode; }

private:
    src::Drivers* drivers;
    DrivetrainSubsystem* drivetrain;
    GimbalSubsystem* gimbal;
    DriveMode driveMode;
    ControlMode controlMode;
    float x, y, r;
};
}  // namespace commands