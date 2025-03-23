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

#include "subsystems/drivetrain/JoystickDriveCommand.hpp"
#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"
#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainFollowTurretCommand.hpp"
#include "util/trigger.hpp"

#include "drivers.hpp"

namespace robots {
class InfantryControl : public ControlInterface {
public:
    // pass drivers back to root robotcontrol to store
    InfantryControl(src::Drivers *drivers) : drivers(drivers), hardware(InfantryHardware{drivers}) {}
    // functions we are using
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
        ui.setDefaultCommand(&draw);

        // unjamButton = Trigger(drivers, [this](){ return this->drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;});

        shootButton.onTrue(&shooterStart).whileTrue(&indexer10Hz);
        unjamButton.onTrue(&shooterStop).whileTrue(&indexerUnjam);

        // //peeking
        // peekLeftButton.whileTrue(&peekLeft);
        // peekRightButton.whileTrue(&peekRight);
        // peekCenterButton.whileTrue(&peekCenter);
        
        // drivers->commandMapper.addMap(&controllerToKeyboardMouseMapping);
        // drivers->commandScheduler.addCommand(&indexerUnjam);
    }

    void update() override {

        shootButton.update();

        // if(unjamButton.getAsBoolean()){
        //     drivers->commandScheduler.addCommand(&indexerUnjam);
        // }
        unjamButton.update();
        // for (Trigger* trigger : triggers) {
        //     trigger->update();
        // }
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

    //CHANGE NUMBERS LATER
    commands::DrivetrainFollowTurretCommand peekRight{drivers, &drivetrain, &gimbal, 1};
    commands::DrivetrainFollowTurretCommand peekLeft{drivers, &drivetrain, &gimbal, -1};
    commands::DrivetrainFollowTurretCommand peekCenter{drivers, &drivetrain, &gimbal, 0};

    commands::JoystickDriveCommand driveCommand{drivers, &drivetrain, &gimbal};

    // mappings


    //maybe change later
    Trigger rightSwitchUp{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP};
    Trigger rightSwitchDown{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN};
    
    Trigger leftSwitchUp{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP};
    Trigger leftSwitchDown{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN};

    //peeking
    Trigger peekLeftButton{drivers, Remote::Key::Q};
    Trigger peekRightButton{drivers, Remote::Key::E};
    // Trigger peekCenterButton = (Trigger(drivers, Remote::Key::Q) & Trigger(drivers, Remote::Key::E)) | Trigger(drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);

    // //shooting
    Trigger shootButton{drivers, Remote::Channel::WHEEL, 0.5};// = Trigger(drivers, Remote::Channel::WHEEL, 0.5) | Trigger(drivers, MouseButton::LEFT);
    // //unjamming
    Trigger unjamButton{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP};// = Trigger(drivers, Remote::Channel::WHEEL, -0.5) | Trigger(drivers, Remote::Key::Z);

    //Trigger indexSpinButton;// = Trigger(drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP) & Trigger(drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    Trigger* triggers[8] = {&rightSwitchUp, &rightSwitchDown, &leftSwitchUp, &leftSwitchDown, &peekLeftButton, &peekRightButton, &shootButton, &unjamButton};//, &indexSpinButton};


    // ToggleCommandMapping controllerToKeyboardMouseMapping{drivers, {&look2}, RemoteMapState({Remote::Key::CTRL, Remote::Key::Z})};

};

}  // namespace robots
