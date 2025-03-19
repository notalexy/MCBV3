#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"


#include "drivers.hpp"

namespace commands
{
using subsystems::DrivetrainSubsystem;
using subsystems::GimbalSubsystem;

using tap::communication::serial::Remote;

class DrivetrainFollowTurretCommand : public tap::control::Command
{
public:
    DrivetrainFollowTurretCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, float angleOffset)
        : drivers(drivers),
          drivetrain(drive),
          gimbal(gimbal),
          angleOffset(angleOffset)
    {
        addSubsystemRequirement(drive);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "drive with joystick command"; }

private:
    src::Drivers* drivers;
    DrivetrainSubsystem* drivetrain;
    GimbalSubsystem* gimbal;
    float angleOffset;

    float x, y, r;
    
};
}  // namespace commands