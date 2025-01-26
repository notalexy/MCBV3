#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"

#include "drivers.hpp"

namespace subsystems
{

static tap::arch::PeriodicMilliTimer shooterControllerTimer(2);
static tap::arch::PeriodicMilliTimer secondTimer(100);
class FlyWheelSubsystem : public tap::control::Subsystem
{
public:  // Public Variables
// constexpr static double PI = 3.14159;
constexpr static int FLYWHEEL_MOTOR_MAX_RPM = 8333;  // We had 5000 last year, and we can go 30/18 times as fast. So 5000 * 30/18
constexpr static tap::algorithms::SmoothPidConfig pid_conf_flywheel = {40, 0.1, 0, 10.0, 10000, 1, 0, 1, 0, 0, 0};

private:  // Private Variables
tap::Drivers* drivers;
// TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction
tap::motor::DjiMotor motor_Flywheel1{drivers, tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS2, true, "Flywheel", 0, 0};
tap::motor::DjiMotor motor_Flywheel2{drivers, tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS2, false, "Flywheel", 0, 0};

tap::algorithms::SmoothPid flywheelPIDController1{pid_conf_flywheel};
tap::algorithms::SmoothPid flywheelPIDController2{pid_conf_flywheel};

int32_t flyWheel1Voltage = 0;
int32_t flyWheel2Voltage = 0;

int targetMotorRPM;

bool robotDisabled = false;

public:  // Public Methods 
    FlyWheelSubsystem(tap::Drivers *drivers);

    ~FlyWheelSubsystem() {}  // Intentionally left blank

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
    void refresh() override;

    void setTargetMotorRPM(int targetMotorRPM);
    /*
        * Call this function to set all Turret motors to 0 desired RPM, calculate the voltage level in which to achieve this quickly
        * and packages this information for the motors TO BE SENT over CanBus
        */
    void stopMotors();

    inline void enable() { this->robotDisabled = false; }
    inline void disable() { this->robotDisabled = true; }

    private:  // Private Methods
    };
}  // namespace ThornBots
