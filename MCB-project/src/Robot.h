#ifndef ROBOT_H_
#define ROBOT_H_

#include <cmath>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "Gimbal/GimbalSubsystem.h"

#include "drivers_singleton.hpp"

namespace ThornBots
{
class Robot
{
private:  // Private Variables
    tap::Drivers* drivers;
    GimbalSubsystem* gimbalSubsystem;

public:  // Public Methods
    Robot(tap::Drivers* driver, GimbalSubsystem* gimbalSubsystem);

    void initialize();

    void update();

    inline void stopRobot()
    {
        // TODO: add motors
        drivers->djiMotorTxHandler.encodeAndSendCanData();
    }
};
}  // namespace ThornBots

#endif  // ROBOT_H_