#include "YawController.hpp"

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

float YawController::calculate(float currentPosition, float currentVelocity, float currentDrivetrainVelocity, float targetPosition, float inputVelocity, float deltaT)
{
    positionError = targetPosition - currentPosition;
    while (positionError > static_cast<float>(M_PI))
    {
        positionError -= static_cast<float>(M_TWOPI);
    }
    while (positionError < static_cast<float>(-M_PI))
    {
        positionError += static_cast<float>(M_TWOPI);
    }

    choiceKDT = currentDrivetrainVelocity * positionError > 0 ? KDT : KDT_REV;  // check if turret is fighting drivetrain;
    inputVelocity = std::clamp(inputVelocity, -VELO_MAX / 2, VELO_MAX / 2);
    float targetVelocity = (KP + signum(currentDrivetrainVelocity) * choiceKDT) * positionError + inputVelocity;
    //experimental
    // targetVelocity = signum(positionError)*sqrt(currentVelocity*currentVelocity + 2 * A_DECEL * abs(positionError));

    // model based motion profile
    aMaxTemp = (C + (KB * KT * RATIO * RATIO) / RA + UK * signum(currentDrivetrainVelocity - currentVelocity));
    maxVelocity = std::min(VELO_MAX, pastTargetVelocity + 1 / J * (aMaxTemp + (VOLT_MAX * KT * RATIO) / RA) * A_SCALE * deltaT);
    minVelocity = std::max(-VELO_MAX, pastTargetVelocity + 1 / J * (aMaxTemp - (VOLT_MAX * KT * RATIO) / RA) * A_SCALE * deltaT);
    targetVelocity = std::clamp(targetVelocity, minVelocity, maxVelocity);

    // velocity controller
    velocityError = targetVelocity - currentVelocity;
    targetRelativeVelocity = targetVelocity - currentDrivetrainVelocity;
    targetAcceleration = (targetVelocity - pastTargetVelocity) / deltaT;
    pastTargetVelocity = targetVelocity;

    // integral velocity controller
    if (abs(pastOutput) < INT_THRESH || velocityError * buildup < 0)  // signs are opposite
    {                                                                 // saturation detection
        // if (velocityError * buildup < 0) {                              // overshooting
        buildup *= (1 - TAKEBACK);  // take back not quite half
        // }    
        buildup += velocityError * deltaT;  // integrate normally
    }
    // calculation for setting target current aka velocity controller
    targetCurrent = KVISC * targetRelativeVelocity + UK * signum(targetRelativeVelocity) + KA * targetAcceleration + KPV * velocityError + KIV * buildup;

    pastOutput = RA * targetCurrent + KV * targetRelativeVelocity;
    return std::clamp(pastOutput, -VOLT_MAX, VOLT_MAX);
}
}  // namespace subsystems
