#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "drivers.hpp"

namespace subsystems
{

class DrivetrainSubsystem : public tap::control::Subsystem
{
public:  // Public Variables
    // constexpr static float PI = 3.14159; //Everyone likes Pi!
    constexpr static tap::algorithms::SmoothPidConfig PID_CONF_DT =
        {20, 0, 0, 0, 18000, 1, 0, 1, 0, 0, 0};
    constexpr static tap::algorithms::SmoothPidConfig PID_CONF_DRIVE_TRAIN_FOLLOWS_TURRET =
        {500, 0.5, 0, 0, 18000, 1, 0, 1, 0, 0, 0};  // TODO: Tune this profile
#if defined(sentry) 
    constexpr static float HIGH_LIM_INC =
        40;  // in watts, however many watts over the current limit given by the ref system
    constexpr static float REG_LIM_INC = 0;        // in watts, should be zero
    float limitIncrease = REG_LIM_INC;  // in watts

    constexpr static float MIN_BUFFER = 10;    // in joules, how much should remain unused in the buffer
                                     // (disables limitIncrease if the buffer is less than this)
    constexpr static float VOLT_MAX = 24;      // Volts
    constexpr static float RA = 0.194 - 0.01;  // ohms //was 1.03 or 0.194
    constexpr static float KB = 0.35 / 19.2;   // volt-rad/s  //0.39
    constexpr static float VELO_LOSS = 0.42;   // magic number representing loss from high rpm
    constexpr static float IDLE_DRAW = 3;      // watts, measured
    constexpr static float DEFAULT_LIMIT = 100;

    constexpr static float KT = 0.35;
#else
    constexpr static float HIGH_LIM_INC =
        40;  // in watts, however many watts over the current limit given by the ref system
    constexpr static float REG_LIM_INC = 0;        // in watts, should be zero
    float limitIncrease = REG_LIM_INC;  // in watts

    constexpr static float MIN_BUFFER = 10;    // in joules, how much should remain unused in the buffer
                                     // (disables limitIncrease if the buffer is less than this)
    constexpr static float VOLT_MAX = 24;      // Volts
    constexpr static float RA = 0.194 - 0.01;  // ohms //was 1.03 or 0.194
    constexpr static float KB = 0.35 / 19.2;   // volt-rad/s  //0.39
    constexpr static float VELO_LOSS = 0.42;   // magic number representing loss from high rpm
    constexpr static float IDLE_DRAW = 3;      // watts, measured
    constexpr static float DEFAULT_LIMIT = 100;

    constexpr static float KT = 0.35;
#endif
    
    

private:                                            // Private Variables
    tap::Drivers* drivers;
    tap::motor::DjiMotor motorOne;
    tap::motor::DjiMotor motorTwo;
    tap::motor::DjiMotor motorThree;
    tap::motor::DjiMotor motorFour;
    tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(PID_CONF_DT);
    tap::algorithms::SmoothPid pidControllerDTFollowsT =
        tap::algorithms::SmoothPid(PID_CONF_DRIVE_TRAIN_FOLLOWS_TURRET);

    float motorOneRPM, motorTwoRPM, motorThreeRPM, motorFourRPM = 0.0, powerLimit = 100;

    float I1t, I2t, I3t, I4t;

   

public:  // Public Methods
    DrivetrainSubsystem(tap::Drivers* driver);
    ~DrivetrainSubsystem() {}  // Intentionally blank

    /*
     * Call this function once, outside of the main loop.
     * This function will initalize all of the motors, timers, pidControllers, and any other used
     * object. If you want to know what initializing actually does, ping Teaney in discord, or just
     * Google it. It's pretty cool.
     */
    void initialize();

    void refresh() override;
    /*
     * Call this function when you want the Turret to follow the DriveTrain
     * Should be called within the main loop, so called every time in the main loop when you want
     * the described behavior. This will allow the drivetrain to translate with left stick, and turn
     * with the right stick or beyblade depending how this is called.
     */
    void setTargetTranslationVector(float translationSpeed, float translationAngle);

    /*
     * Call this function to convert the desired RPM for all of motors in the DrivetrainSubsystem to
     * a voltage level which would then be sent over CanBus to each of the motor controllers to
     * actually set this voltage level on each of the motors. Should be placed inside of the main
     * loop, and called periodically, every
     */
    void setMotorSpeeds();

    /*
     * Call this function to set all DriveTrain motors to 0 desired RPM. CALL setMotorSpeeds() FOR
     * THIS TO WORK
     */
    void stopMotors();

private:  // Private Methods
    /*
     * Call this function to calculate and OVERWRITE DriveTrain motors' RPMs to translate at the
     * given magnitude and angle Angle should be in radians, with 0 being straight forward relative
     * to the drivetrain and the positive direction being CCW. (i.e. So to translate directly to the
     * left, relative to the drivetrain, you would call this function as:
     * convertTranslationSpeedToMotorSpeeds(0.75, pi/2);)
     */
    void convertTranslationSpeedToMotorSpeeds(float magnitude, float angle);

    /*
     * Call this function to calculate and ADJUST DriveTrain motors' RPMs to rotate at the given
     * turnSpeed. This input is unitless, should be from [0, 1], and simply multiplies it by a
     * constant, adjustable maximum factor of maximum speed.
     */
    void adjustMotorSpeedWithTurnSpeed(float turnSpeed);
    

};
}  // namespace subsystems