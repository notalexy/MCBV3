#include "Robot.h"

namespace ThornBots
{
Robot::Robot(tap::Drivers* driver, GimbalSubsystem* gimbalSubsystem)
{
    // TODO:
}

void Robot::initialize()
{
    // TODO:
}

void Robot::update()
{
    // TODO:

    //  DO NOT REMOVE
    if (!drivers->remote.isConnected())
    {
        // I WILL COME FOR U IN YOUR SLEEP if you del ;)
        stopRobot();
        return;
    }
}

}  // namespace ThornBots