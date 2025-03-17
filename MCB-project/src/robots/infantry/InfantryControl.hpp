#include "robots/RobotControl.hpp"

#if defined(INFANTRY)
#include "robots/infantry/InfantryHardware.hpp"
#else
#include "robots/infantry/MechInfantryHardware.hpp"
#endif

#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/ui/UIDrawCommand.hpp"

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
        hardware(InfantryHardware{drivers}) {}
    //functions we are using
    void initialize() override {

        // Initialize subsystems
        gimbal.initialize();
        flywheel.initialize();
        indexer.initialize();
        drivetrain.initialize();
        ui.initialize();

        // Register subsystems;
        drivers->commandScheduler.registerSubsystem(&gimbal);
        drivers->commandScheduler.registerSubsystem(&flywheel);
        drivers->commandScheduler.registerSubsystem(&indexer);
        drivers->commandScheduler.registerSubsystem(&drivetrain);
        drivers->commandScheduler.registerSubsystem(&ui);

        // Run startup commands
        gimbal.setDefaultCommand(&look);
        flywheel.setDefaultCommand(&shooterStop);
        drivetrain.setDefaultCommand(&driveCommand);

        // drivers->commandMapper.addMap(&startShootMapping);
        // drivers->commandMapper.addMap(&idleShootMapping);
        // drivers->commandMapper.addMap(&stopShootMapping);
        drivers->commandMapper.addMap(&controllerToKeyboardMouseMapping);

    }

    src::Drivers *drivers;
    InfantryHardware hardware;

    // Subsystems
    subsystems::UISubsystem ui{drivers};
    subsystems::GimbalSubsystem gimbal{drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::FlywheelSubsystem flywheel{drivers, &hardware.flywheelMotor1, &hardware.flywheelMotor2};
    subsystems::IndexerSubsystem indexer{drivers, &hardware.indexMotor};
    subsystems::DrivetrainSubsystem drivetrain{drivers, &hardware.driveMotor1, &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};

    // //commands
    commands::UIDrawCommand draw{&ui};

    commands::JoystickMoveCommand look{drivers, &gimbal};
    commands::MouseMoveCommand look2{drivers, &gimbal};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 10};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

   commands::JoystickDriveCommand driveCommand{drivers, &drivetrain, &gimbal};

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
