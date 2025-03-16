#include "robots/RobotControl.hpp"

#if defined(INFANTRY)
#include "robots/infantry/InfantryHardware.hpp"
#else
#include "robots/infantry/MechInfantryHardware.hpp"
#endif

#include "subsystems/ui/UISubsystem.hpp"

#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"

#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"

#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"

#include "subsystems/drivetrain/JoystickDriveCommand.hpp"

#include "drivers.hpp"


namespace robots
{
class InfantryControl : public ControlInterface
{
public:
    //pass drivers back to root robotcontrol to store
    InfantryControl(src::Drivers *drivers) :
        drivers(drivers),
        gimbal(&hardware.gimbal),
        flywheel(&hardware.flywheel),
        indexer(&hardware.indexer),
        drivetrain(&hardware.drivetrain) {}
    //functions we are using
    void initialize() override {

        // Initialize subsystems
        gimbal->initialize();
        flywheel->initialize();
        indexer->initialize();
        drivetrain->initialize();
        ui.initialize();

        // Register subsystems;
        drivers->commandScheduler.registerSubsystem(gimbal);
        drivers->commandScheduler.registerSubsystem(flywheel);
        drivers->commandScheduler.registerSubsystem(indexer);
        drivers->commandScheduler.registerSubsystem(drivetrain);
        drivers->commandScheduler.registerSubsystem(&ui);

        // Run startup commands
        gimbal->setDefaultCommand(&look);
        flywheel->setDefaultCommand(&shooterStop);
        drivetrain->setDefaultCommand(&driveCommand);

        // drivers->commandMapper.addMap(&startShootMapping);
        // drivers->commandMapper.addMap(&idleShootMapping);
        // drivers->commandMapper.addMap(&stopShootMapping);
        drivers->commandMapper.addMap(&controllerToKeyboardMouseMapping);

    }

    src::Drivers *drivers;
    InfantryHardware hardware{drivers};

    // Subsystems
    subsystems::UISubsystem ui{drivers};
    subsystems::GimbalSubsystem *gimbal;
    subsystems::FlywheelSubsystem *flywheel;
    subsystems::DrivetrainSubsystem *drivetrain;
    subsystems::IndexerSubsystem *indexer;

    // //commands
    commands::JoystickMoveCommand look{drivers, gimbal};
    commands::MouseMoveCommand look2{drivers, gimbal};

    commands::ShooterStartCommand shooterStart{drivers, flywheel};
    commands::ShooterStopCommand shooterStop{drivers, flywheel};

    commands::IndexerNBallsCommand indexer10Hz{drivers, indexer, -1, 10};
    commands::IndexerUnjamCommand indexerUnjam{drivers, indexer};

   commands::JoystickDriveCommand driveCommand{drivers, drivetrain, gimbal};

    //mappings
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

}
