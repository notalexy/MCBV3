#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"

#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"

#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"

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
       indexer.initialize();

        // Register subsystems;
        drivers->commandScheduler.registerSubsystem(&gimbal);
        drivers->commandScheduler.registerSubsystem(&flywheel);
        drivers->commandScheduler.registerSubsystem(&indexer);

        // Run startup commands
        gimbal.setDefaultCommand(&look);
        flywheel.setDefaultCommand(&shooterStop);


        drivers->commandMapper.addMap(&startShootMapping);
        drivers->commandMapper.addMap(&idleShootMapping);
        drivers->commandMapper.addMap(&stopShootMapping);
        drivers->commandMapper.addMap(&controllerToKeyboardMouseMapping);
    }

    src::Drivers* drivers;

    // Subsystems
    subsystems::GimbalSubsystem gimbal{drivers};

    commands::JoystickMoveCommand look{drivers, &gimbal};
    commands::MouseMoveCommand look2{drivers, &gimbal};

    subsystems::FlyWheelSubsystem flywheel{drivers};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    subsystems::IndexerSubsystem indexer{drivers};

    commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 10};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

    ToggleCommandMapping controllerToKeyboardMouseMapping {
        drivers,
        {&look2},
        RemoteMapState({Remote::Key::CTRL, Remote::Key::Z})};

    HoldCommandMapping startShootMapping {
        drivers,
        {&indexer10Hz},
        RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)};

    HoldCommandMapping idleShootMapping {
        drivers,
        {&shooterStart},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)};

    HoldCommandMapping stopShootMapping {
        drivers,
        {&indexerUnjam, &shooterStop},
        RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN)};

    HoldCommandMapping startShootMappingMouse {
        drivers,
        {&shooterStart, &indexer10Hz},
        RemoteMapState(RemoteMapState::MouseButton::LEFT)
    };
    
    HoldCommandMapping stopShootMappingMouse {
        drivers,
        {&shooterStop, &indexerUnjam},
        RemoteMapState(RemoteMapState::MouseButton::LEFT)
    };
};