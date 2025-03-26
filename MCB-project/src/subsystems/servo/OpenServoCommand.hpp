#pragma once

#include "tap/control/command.hpp"

#include "subsystems/servo/ServoSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::OpenServoCommand;

class OpenServoCommand : public tap::control::Command
{
public:
OpenServoCommand(src::Drivers* drivers, ServoSubsystem* servo)
        : drivers(drivers),
          servo(servo)
    {
        addSubsystemRequirement(servo);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "open servo command"; }

private:
    src::Drivers* drivers;
    ServoSubsystem* servo;
};
}  // namespace commands