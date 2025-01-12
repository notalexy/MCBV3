#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::DrivetrainSubsystem;
using tap::communication::serial::Remote;

class JoystickDriveCommand : public tap::control::Command
{
public:
    JoystickDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive)
        : drivers(drivers),
          drivetrain(drive)
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

    double translationAngle, translationSpeed;
    double x, y, r;
    
};
}  // namespace commands