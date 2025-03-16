#include "robots/RobotControl.hpp"
#include "robots/sentry/SentryHardware.hpp"

#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"

#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"

#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"

#include "drivers.hpp"


namespace robots
{
class SentryControl : public ControlInterface
{
public:
    //pass drivers back to root robotcontrol to store
    SentryControl(SentryHardware *hardware) : hardware(hardware) {}
    //functions we are using
    void initialize() override {

        // Initialize subsystems
        hardware->gimbal.initialize();
        hardware->flywheel.initialize();
        hardware->indexer.initialize();
        hardware->drivetrain.initialize();

        // Register subsystems;
        hardware->drivers->commandScheduler.registerSubsystem(&hardware->gimbal);
        hardware->drivers->commandScheduler.registerSubsystem(&hardware->flywheel);
        hardware->drivers->commandScheduler.registerSubsystem(&hardware->indexer);
        hardware->drivers->commandScheduler.registerSubsystem(&hardware->drivetrain);

        // Run startup commands
        hardware->gimbal.setDefaultCommand(&look);
        hardware->flywheel.setDefaultCommand(&shooterStop);


        hardware->drivers->commandMapper.addMap(&startShootMapping);
        hardware->drivers->commandMapper.addMap(&idleShootMapping);
        hardware->drivers->commandMapper.addMap(&stopShootMapping);
        hardware->drivers->commandMapper.addMap(&controllerToKeyboardMouseMapping);

    }

    SentryHardware *hardware;

    // Subsystems


    //commands
    commands::JoystickMoveCommand look{hardware->drivers, &hardware->gimbal};
    commands::MouseMoveCommand look2{hardware->drivers, &hardware->gimbal};

    commands::ShooterStartCommand shooterStart{hardware->drivers, &hardware->flywheel};
    commands::ShooterStopCommand shooterStop{hardware->drivers, &hardware->flywheel};

    commands::IndexerNBallsCommand indexer10Hz{hardware->drivers, &hardware->indexer, -1, 10};
    commands::IndexerUnjamCommand indexerUnjam{hardware->drivers, &hardware->indexer};

    //mappings
    ToggleCommandMapping controllerToKeyboardMouseMapping {
        hardware->drivers,
        {&look2},
        RemoteMapState({Remote::Key::CTRL, Remote::Key::Z})};

    HoldCommandMapping startShootMapping {
        hardware->drivers,
        {&indexer10Hz},
        RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)};

    HoldCommandMapping idleShootMapping {
        hardware->drivers,
        {&shooterStart},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)};

    HoldCommandMapping stopShootMapping {
        hardware->drivers,
        {&indexerUnjam, &shooterStop},
        RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN)};

    HoldCommandMapping startShootMappingMouse {
        hardware->drivers,
        {&shooterStart, &indexer10Hz},
        RemoteMapState(RemoteMapState::MouseButton::LEFT)
    };
    
    HoldCommandMapping stopShootMappingMouse {
        hardware->drivers,
        {&shooterStop, &indexerUnjam},
        RemoteMapState(RemoteMapState::MouseButton::LEFT)
    };
};

}
