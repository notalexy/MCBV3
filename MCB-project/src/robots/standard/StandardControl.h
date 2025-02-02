#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "subsystems/gimbal/GimbalSubsystem.h"
#include "subsystems/gimbal/JoystickMoveCommand.h"
#include "subsystems/gimbal/MouseMoveCommand.h"

#include "subsystems/flywheel/FlywheelSubsystem.h"
#include "subsystems/flywheel/ShooterStartCommand.h"
#include "subsystems/flywheel/ShooterStopCommand.h"

#include "subsystems/indexer/IndexerSubsystem.h"
#include "subsystems/indexer/IndexerNBallsCommand.h"
#include "subsystems/indexer/IndexerUnjamCommand.h"

#include "drivers.h"

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
    }

    src::Drivers* drivers;

    // Subsystems
    subsystems::GimbalSubsystem gimbal{drivers};

    // commands::JoystickMoveCommand look{drivers, &gimbal};
    commands::MouseMoveCommand look{drivers, &gimbal};

    subsystems::FlyWheelSubsystem flywheel{drivers};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    subsystems::IndexerSubsystem indexer{drivers};

    commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 10};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

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
};