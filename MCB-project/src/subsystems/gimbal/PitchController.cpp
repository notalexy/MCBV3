#include "PitchController.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include <bits/stdc++.h>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

namespace subsystems
{
PitchController::PitchController() {}

float PitchController::calculate(float currentPosition, float currentVelocity, float targetPosition, float deltaT)
{
    float positionError = targetPosition - currentPosition;

    float targetVelocity = KP * positionError;

    targetPos = targetPosition;
    currentPos = currentPosition;

    // model based motion profile
    float maxVelocity = std::min(VELO_MAX, pastTargetVelocity + ACCEL_MAX * deltaT);
    float minVelocity = std::max(-VELO_MAX, pastTargetVelocity - ACCEL_MAX * deltaT);
    targetVelocity = std::clamp(targetVelocity, minVelocity, maxVelocity);

    currentVelo = currentVelocity;
    targetVelo = targetVelocity;

    // velocity controller
    float velocityError = targetVelocity - currentVelocity;
    pastTargetVelocity = targetVelocity;

    // integral velocity controller
    if (abs(pastOutput) < INT_THRESH || velocityError * buildup < 0)
    {  // saturation detection
        if (velocityError * buildup < 0)
        {                               // overshooting
            buildup *= (1 - TAKEBACK);  // take back not quite half
        }
        else
        {
            buildup += velocityError * deltaT;  // integrate normally
        }
    }
    // calculation for setting target current aka velocity controller
    float targetCurrent = KSTATIC * signum(targetVelocity) + KF + KPV * velocityError + KIV * buildup;

    pastOutput = RA * targetCurrent + KV * targetVelocity;
    output2 = pastOutput * 100;
    return std::clamp(pastOutput, -VOLT_MAX, VOLT_MAX);
}
}  // namespace subsystems
