#include "Robot.h"

#include <cmath>

namespace ThornBots {

    double yawEncoderValue, IMUAngle = 0.0;
    /*
     * Constructor for Robot
     */
    Robot::Robot(tap::Drivers* driver, GimbalSubsystem* gimbalSubsystem) {
        this->drivers = driver;
        this->gimbalSubsystem = gimbalSubsystem;
    }

    void Robot::initialize() {
        Board::initialize();
        drivers->can.initialize();
        drivers->bmi088.initialize(500, 0.0, 0.0);
        drivers->bmi088.requestRecalibration();
        drivers->remote.initialize();
        gimbalSubsystem->initialize();

        modm::delay_ms(2500);  // Delay 2.5s to allow the IMU to turn on and get working before we move it around
        // TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's
        // reading correctly, ect)
        imuOffset = gimbalSubsystem->getYawEncoderValue();
        targetYawAngleWorld += yawAngleRelativeWorld;
    }

    void Robot::update() {
        drivers->canRxHandler.pollCanData();

        drivers->remote.read();  // Reading the remote before we check if it is connected yet or not.
        if (IMUTimer.execute()) {
            drivers->bmi088.periodicIMUUpdate();
        }

        right_stick_horz = drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
        right_stick_vert = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
        // Turning the remote raw values into values we can use more easily (circular cordinates)
        // STOP Updating stick values

        driveTrainRPM = 0;  // TODO: get this. Either power from DT motors, using yaw encoder and IMU,
                            // or something else
        yawRPM = PI / 180 * drivers->bmi088.getGz();
        yawAngleRelativeWorld = fmod(PI / 180 * drivers->bmi088.getYaw() - imuOffset, 2 * PI);

        if (drivers->remote.isConnected())
            enableRobot();
        else
            disableRobot();

        if (robotDisabled) return;

        if (motorsTimer.execute()) {
            gimbalSubsystem->setMotorSpeeds();
        }

        if (updateInputTimer.execute()) {
            double temp = right_stick_horz * YAW_TURNING_PROPORTIONAL;
            driveTrainEncoder = gimbalSubsystem->getYawEncoderValue();
            yawEncoderCache = driveTrainEncoder;
            targetYawAngleWorld += temp;

            targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);
            gimbalSubsystem->turretMove(targetYawAngleWorld,
                                        0.1 * PI * right_stick_vert,  // was - 0.5 * PI
                                        driveTrainRPM, yawAngleRelativeWorld, yawRPM, temp / dt, dt);
        }
    }

}  // namespace ThornBots