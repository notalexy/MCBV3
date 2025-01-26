#include "YawController.h"

#include <cmath>
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

namespace subsystems
{
YawController::YawController() {}

float YawController::calculate(
    float currentPosition,
    float currentVelocity,
    float currentDrivetrainVelocity,
    float targetPosition,
    float inputVelocity,
    float deltaT)
{
    float positionError = targetPosition - currentPosition;
    while (positionError > static_cast<float>(M_PI))
    {
        positionError -= static_cast<float>(M_TWOPI);
    }
    while (positionError < static_cast<float>(-M_PI))
    {
        positionError += static_cast<float>(M_TWOPI);
    }

    float choiceKDT = currentDrivetrainVelocity * positionError > 0
                           ? KDT
                           : KDT_REV;  // check if turret is fighting drivetrain;
    inputVelocity = std::clamp(inputVelocity, -VELO_MAX / 2, VELO_MAX / 2);
    float targetVelocity =
        (KP + signum(currentDrivetrainVelocity) * choiceKDT) * positionError + inputVelocity;

    // model based motion profile
    float aMaxTemp =
        (C + (KB * KT * RATIO*RATIO) / RA +
         UK * signum(currentDrivetrainVelocity - currentVelocity));
    float maxVelocity = std::min(
        VELO_MAX,
        pastTargetVelocity + 1 / J * (aMaxTemp + (VOLT_MAX * KT * RATIO) / RA) * A_SCALE * deltaT);
    float minVelocity = std::max(
        -VELO_MAX,
        pastTargetVelocity + 1 / J * (aMaxTemp - (VOLT_MAX * KT * RATIO) / RA) * A_SCALE * deltaT);
    targetVelocity = std::clamp(targetVelocity, minVelocity, maxVelocity);

    // velocity controller
    float velocityError = targetVelocity - currentVelocity;
    float targetRelativeVelocity = targetVelocity - currentDrivetrainVelocity;
    float targetAcceleration = (targetVelocity - pastTargetVelocity) / deltaT;
    pastTargetVelocity = targetVelocity;

    // integral velocity controller
    if (abs(pastOutput) < INT_THRESH || velocityError * buildup < 0)
    {   // saturation detection
        // if (velocityError * buildup < 0) {                              // overshooting
        buildup *= (1 - TAKEBACK);  // take back not quite half
        // }
        buildup += velocityError * deltaT;  // integrate normally
    }
    // calculation for setting target current aka velocity controller
    float targetCurrent = KVISC * targetRelativeVelocity + UK * signum(targetRelativeVelocity) +
                           KA * targetAcceleration + KPV * velocityError + KIV * buildup;

    pastOutput = RA * targetCurrent + KV * targetRelativeVelocity;
    return std::clamp(pastOutput, -VOLT_MAX, VOLT_MAX);
}
}  // namespace ThornBots
