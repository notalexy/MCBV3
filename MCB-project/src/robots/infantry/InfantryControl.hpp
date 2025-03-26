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

#include "subsystems/drivetrain/DrivetrainDriveCommand.hpp"
#include "subsystems/drivetrain/DrivetrainStopCommand.hpp"
#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"
#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"
#include "subsystems/indexer/IndexerStopCommand.hpp"
#include "subsystems/ui/UISubsystem.hpp"
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
        drivetrain.setDefaultCommand(&stopDriveCommand);
        indexer.setDefaultCommand(&indexerStopCommand);
        ui.setDefaultCommand(&draw);

        // unjamButton = Trigger(drivers, [this](){ return this->drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;});

        shootButton.onTrue(&shooterStart)->whileTrue(&indexer10Hz);
        unjamButton.onTrue(&shooterStop)->whileTrue(&indexerUnjam);

        //peeking
        peekLeftButton.whileTrue(&peekLeft);
        peekRightButton.whileTrue(&peekRight);

        //Mouse and Keyboard mappings
        unjamKey.whileTrue(&indexerUnjam)->onTrue(&shooterStop);
        //implement autoaim
        shootKey.whileTrue(&indexer10Hz)->onTrue(&shooterStart);
        //implement speed mode
        //implement beyblade types
        beybladeType0Key.onTrue(&drivetrainFollowKeyboard);
        beybladeType1Key.onTrue(&beybladeSlowKeyboard);
        beybladeType2Key.onTrue(&beybladeFastKeyboard);

        toggleUIKey.toggleOnFalse(&draw);
        drivers->commandScheduler.addCommand(&draw);
        //implement toggle UI
        
        joystickDrive0.whileTrue(&drivetrainFollowKeyboard);
        joystickDrive1.whileTrue(&drivetrainFollowJoystick);
        joystickDrive2.whileTrue(&beybladeJoystick);

        // drivers->commandMapper.addMap(&controllerToKeyboardMouseMapping);
        // drivers->commandScheduler.addCommand(&indexerUnjam);
    }

    void update() override {

        for (Trigger* trigger : triggers) {
            trigger->update();
        }
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
    commands::UIDrawCommand draw{&ui, &gimbal, &flywheel, &indexer, &drivetrain};

    commands::JoystickMoveCommand look{drivers, &gimbal};
    commands::MouseMoveCommand look2{drivers, &gimbal};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 10};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};
    commands::IndexerStopCommand indexerStopCommand{drivers, &indexer};

    //CHANGE NUMBERS LATER
    commands::DrivetrainDriveCommand peekRight{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_RIGHT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekLeft{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_LEFT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeSlowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand beybladeFastKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE2, commands::ControlMode::KEYBOARD};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    // mappings


    //maybe change later

    //peeking
    Trigger peekLeftButton{drivers, Remote::Key::Q};
    Trigger peekRightButton{drivers, Remote::Key::E};
    Trigger joystickDrive0{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP};// = (Trigger(drivers, Remote::Key::Q) & Trigger(drivers, Remote::Key::E)) | Trigger(drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    Trigger joystickDrive1{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID};
    Trigger joystickDrive2{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN};
    //shooting
    Trigger shootButton{drivers, Remote::Channel::WHEEL, 0.5};// = Trigger(drivers, Remote::Channel::WHEEL, 0.5) | Trigger(drivers, MouseButton::LEFT);
    //unjamming we know the button reacts 
    Trigger unjamButton{drivers, Remote::Channel::WHEEL, -0.5};// = Trigger(drivers, Remote::Channel::WHEEL, -0.5) | Trigger(drivers, MouseButton::RIGHT);

    //find keyboard unjam button 
    //figure out how to switch kb/m and controller
    //find speed mode button
    //buttons to change beyblade type (variable, fast, none) (3 buttons)
    //find auto aim button for both kb/m and controller
    //

    Trigger unjamKey{drivers, Remote::Key::Z}; //or R if based
    Trigger autoAimKey{drivers, MouseButton::RIGHT};
    Trigger shootKey{drivers, MouseButton::LEFT};
    Trigger speedModeKey{drivers, Remote::Key::SHIFT};
    Trigger beybladeType0Key{drivers, Remote::Key::X};
    Trigger beybladeType1Key{drivers, Remote::Key::C};
    Trigger beybladeType2Key{drivers, Remote::Key::V};
    Trigger toggleUIKey{drivers, Remote::Key::G};

    //Trigger indexSpinButton;// = Trigger(drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP) & Trigger(drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    Trigger* triggers[15] = {&peekLeftButton, &peekRightButton, &joystickDrive0, &joystickDrive1, &joystickDrive2, &shootButton, &unjamButton, &unjamKey, &shootKey, &autoAimKey, &speedModeKey, &beybladeType0Key, &beybladeType1Key, &beybladeType2Key, &toggleUIKey};//, &indexSpinButton};


    // ToggleCommandMapping controllerToKeyboardMouseMapping{drivers, {&look2}, RemoteMapState({Remote::Key::CTRL, Remote::Key::Z})};

};

}  // namespace robots
