#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/gimbal/JoystickMoveCommand.hpp"

#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"

#include "drivers.hpp"

using namespace tap::control;
using namespace tap::communication::serial;

class StandardControl
{
public:
    StandardControl(src::Drivers* drivers) : drivers(drivers) {}

    void initialize()
    {
        // Initialize subsystems
        gimbal.initialize();
        flywheel.initialize();

        // Register subsystems
        drivers->commandScheduler.registerSubsystem(&gimbal);
        drivers->commandScheduler.registerSubsystem(&flywheel);

        // Run startup commands
        gimbal.setDefaultCommand(&look);
        flywheel.setDefaultCommand(&shooterStop);
    }

    src::Drivers* drivers;

    // Subsystems
    subsystems::GimbalSubsystem gimbal{drivers};

    commands::JoystickMoveCommand look{drivers, &gimbal};

    subsystems::FlyWheelSubsystem flywheel{drivers};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};
};