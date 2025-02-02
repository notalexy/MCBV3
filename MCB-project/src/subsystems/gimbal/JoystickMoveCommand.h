#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/gimbal/GimbalSubsystem.h"

#include "drivers.h"

namespace commands
{
using subsystems::GimbalSubsystem;
using tap::communication::serial::Remote;

class JoystickMoveCommand : public tap::control::Command
{
public:
    JoystickMoveCommand(src::Drivers* drivers, GimbalSubsystem* gimbal)
        : drivers(drivers),
          gimbal(gimbal)
    {
        addSubsystemRequirement(gimbal);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "move turret joystick command"; }

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

    bool isCalibrated = false;

    float yaw = 0.0f;
    float pitch = 0.0f;
};
}  // namespace commands