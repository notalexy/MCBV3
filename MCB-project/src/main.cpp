#include "Gimbal/GimbalSubsystem.h"
#include "Robot.h"
#include "drivers_singleton.hpp"

static tap::arch::PeriodicMicroTimer RunTimer(10);  // Don't ask me why. This only works as a global. #Certified Taproot Moment

int main() {
    src::Drivers* drivers = src::DoNotUse_getDrivers();
    ThornBots::GimbalSubsystem* gimbalSubsystem = new ThornBots::GimbalSubsystem(drivers);
    ThornBots::Robot* robot = new ThornBots::Robot(drivers, gimbalSubsystem);

    robot->initialize();
    while (1) {
        if (RunTimer.execute()) {  // Calling this function every 10 us at max
            robot->update();
        }
    }
}