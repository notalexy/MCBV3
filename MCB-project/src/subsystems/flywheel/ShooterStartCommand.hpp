#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/flywheel/FlywheelSubsystem.hpp"

#include "drivers.hpp"
using subsystems::FlywheelSubsystem;
using tap::communication::serial::Remote;

namespace commands
{


class ShooterStartCommand : public tap::control::Command
{
public:
    ShooterStartCommand(src::Drivers* drivers, FlywheelSubsystem* flywheel)
        : drivers(drivers),
          flywheel(flywheel)
    {
        addSubsystemRequirement(flywheel);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "start flywheel command"; }

private:
    src::Drivers* drivers;
    FlywheelSubsystem* flywheel;

};
}  // namespace commands