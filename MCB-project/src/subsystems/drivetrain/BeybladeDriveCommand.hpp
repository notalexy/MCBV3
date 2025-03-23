#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainDriveCommand.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"

#include "drivers.hpp"

namespace commands {
using subsystems::DrivetrainSubsystem;
using subsystems::GimbalSubsystem;

using tap::communication::serial::Remote;


class BeybladeDriveTemplate : public DrivetrainDriveCommand {
public:
    BeybladeDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal)
     : DrivetrainDriveCommand(drivers, drive, gimbal), mode(mode) {}
    virtual Pose2d getDrive() override;

    void setDriveMode(DriveMode newMode) { mode = newMode; }

private:
    float x, y, r;
    DriveMode mode;
};

}  // namespace commands