#pragma once
#include <cmath>
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

namespace subsystems
{
class ChassisController
{
public:
    ChassisController();
    //~YawController();
    double calculate();

private:
    // START getters and setters
 

    double signum(double num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }
};
}  // namespace ThornBots
