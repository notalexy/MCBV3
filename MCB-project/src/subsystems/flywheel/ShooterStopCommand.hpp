#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/flywheel/FlywheelSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::FlyWheelSubsystem;
using tap::communication::serial::Remote;

class ShooterStopCommand : public tap::control::Command
{
public:
    ShooterStopCommand(src::Drivers* drivers, FlyWheelSubsystem* flywheel)
        : drivers(drivers),
          flywheel(flywheel)
    {
        addSubsystemRequirement(flywheel);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "stop flywheel command"; }

private:
    src::Drivers* drivers;
    FlyWheelSubsystem* flywheel;

};
}  // namespace commands