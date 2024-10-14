#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/control/subsystem.hpp"

#include "PitchController.h"
#include "YawController.h"

#include "drivers.hpp"

namespace subsystems
{

static tap::arch::PeriodicMilliTimer turretControllerTimer(2);
class GimbalSubsystem : public tap::control::Subsystem
{
public:  // Public Variables
    // constexpr static double PI = 3.14159;
    constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
    constexpr static int YAW_MOTOR_MAX_VOLTAGE =
        24000;  // Should be the voltage of the battery. Unless the motor maxes out below that.
                // //TODO: Check the datasheets

    static constexpr double YAW_TURNING_PROPORTIONAL = -0.02;
    static constexpr double dt = 0.002;

private:  // Private Variables
    tap::Drivers* drivers;
    // TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction
    tap::motor::DjiMotor motor_Yaw;
    tap::motor::DjiMotor motor_Pitch;

    YawController yawController = YawController();
    PitchController pitchController = PitchController();

    double pitchMotorVoltage, yawMotorVoltage;

    bool robotDisabled = false;

    double right_stick_horz, right_stick_vert = 0;
    double driveTrainRPM, yawRPM, yawAngleRelativeWorld = 0.0, imuOffset;
    bool useKeyboardMouse = false;
    double yawEncoderCache = 0;
    double desiredYawAngleWorld, desiredYawAngleWorld2, driveTrainEncoder = 0.0;
    double stickAccumulator = 0, targetYawAngleWorld = PI,
           targetDTVelocityWorld = 0;  // changed targetYawAngleWorld from 0 to PI

public:  // Public Methods
    GimbalSubsystem(tap::Drivers* drivers);
      
    //~GimbalSubsystem() {}  // Intentionally left blank

    /*
     * Call this function once, outside of the main loop.
     * This function will initalize all of the motors, timers, pidControllers, and any other used
     * object. If you want to know what initializing actually does, ping Teaney in discord, or just
     * Google it. It's pretty cool.
     */
void initialize();

    /*
     * reads the right joystick values and updates the internal values of where the gimbal needs to
     * go
     */
    void update();

    /*
     * Should be called within the main loop, so called every time in the main loop when you want
     * the described behavior. This will allow the drivetrain to translate with the left stick, and
     * the right stick is for the turret. This function should be called when the right switch is in
     * the Down state. Enabling beyblading (left switch is not down) will override this state, and
     * left stick will control drivetrain translating and right stick will control pitch and yaw of
     * the turret.
     */
    void turretMove(
        double desiredYawAngle,
        double desiredPitchAngle,
        double driveTrainRPM,
        double yawAngleRelativeWorld,
        double yawRPM,
        double inputVel,
        double dt);

    /*
     * tells the motors to move the gimbal to its specified angle calculated in update();
     */
    void updateMotors();

    /*
     * Call this function to convert the desired RPM for all of motors in the GimbalSubsystem to a
     * voltage level which would then be sent over CanBus to each of the motor controllers to
     * actually set this voltage level on each of the motors. Should be placed inside of the main
     * loop, and called every time through the loop, ONCE
     */
    void setMotorSpeeds();

    /*
     * Call this function to set all Turret motors to 0 desired RPM, calculate the voltage level in
     * which to achieve this quickly and packages this information for the motors TO BE SENT over
     * CanBus
     */
    void stopMotors();
    inline void disable() { robotDisabled = true; }
    inline void enable() { robotDisabled = false; }

    /*
     * Call this function (any number of times) to reZero the yaw motor location. This will be used
     * when first turning on the robot and setting the Turret to where the front of the DriveTrain
     * is. This function should be called when either in the bootup sequence, or when some,
     * undetermined button is pressed on the keyboard.
     */
    void reZeroYaw();

    inline double getYawEncoderValue()
    {
        return tap::motor::DjiMotor::encoderToDegrees(motor_Yaw.getEncoderUnwrapped()) * PI / 180;
    }
    inline double getPitchEncoderValue()
    {
        return tap::motor::DjiMotor::encoderToDegrees(motor_Pitch.getEncoderUnwrapped()) * PI / 180;
    }
    inline double getYawVel() { return motor_Yaw.getShaftRPM() * PI / 30; }
    inline double getPitchVel() { return motor_Pitch.getShaftRPM() * PI / 30; }

private:  // Private Methods
    int getPitchVoltage(double targetAngle, double dt);
    int getYawVoltage(
        double driveTrainRPM,
        double yawAngleRelativeWorld,
        double yawRPM,
        double desiredAngleWorld,
        double inputVel,
        double dt);
};
}  // namespace ThornBots