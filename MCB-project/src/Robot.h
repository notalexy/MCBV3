#pragma once

#include <cmath>

#include "tap/algorithms/smooth_pid.hpp"

#include "GimbalSubsystem.h"
#include "drivers_singleton.hpp"

namespace ThornBots {
    using namespace tap::communication::serial;

    // Don't ask me why. Timers only work when global. #Certified taproot Moment
    static tap::arch::PeriodicMilliTimer motorsTimer(2);
    static tap::arch::PeriodicMilliTimer IMUTimer(2);
    static tap::arch::PeriodicMilliTimer updateInputTimer(2);
    class Robot {
    public:  // Public Variables
        static constexpr double YAW_TURNING_PROPORTIONAL = -0.02;
        static constexpr double dt = 0.002;

        // static constexpr double
    private:  // Private Variables
        tap::Drivers* drivers;
        GimbalSubsystem* gimbalSubsystem;
        double right_stick_horz, right_stick_vert = 0;
        double driveTrainRPM, yawRPM, yawAngleRelativeWorld = 0.0, imuOffset;
        bool useKeyboardMouse = false;
        double yawEncoderCache = 0;
        double desiredYawAngleWorld, desiredYawAngleWorld2, driveTrainEncoder = 0.0;
        double stickAccumulator = 0, targetYawAngleWorld = PI,
               targetDTVelocityWorld = 0;  // changed targetYawAngleWorld from 0 to PI
        bool robotDisabled = false;


    public:  // Public Methods
        Robot(tap::Drivers* driver, GimbalSubsystem* turretController);

        void initialize();

        void update();

        inline void stopRobot() {
            gimbalSubsystem->stopMotors();
            robotDisabled = true;
        }

        inline void disableRobot() {
            stopRobot();
            gimbalSubsystem->disable();
        }

        inline void enableRobot() {
            robotDisabled = false;
            gimbalSubsystem->enable();
        }

        bool toggleKeyboardAndMouse();

    private:  // Private Methods
   
    };
}  // namespace ThornBots