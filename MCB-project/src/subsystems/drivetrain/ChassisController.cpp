#include "ChassisController.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

namespace subsystems
{
ChassisController::ChassisController() {}

float ChassisController::calculate()
{
   return 0;
}
}  // namespace ThornBots
