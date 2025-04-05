#include "YawController.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "YawControllerConstants.hpp"

namespace subsystems {
YawController::YawController() {}

float YawController::calculate(float currentPosition, float currentVelocity, float currentDrivetrainVelocity, float targetPosition, float inputVelocity, float deltaT) {
    currentVelocity /= PI;
    estimateState(&currentPosition, &currentVelocity, pastTorque, currentDrivetrainVelocity);

    float positionError = targetPosition - currentPosition;
    while (positionError > static_cast<float>(M_PI)) {
        positionError -= static_cast<float>(M_TWOPI);
    }
    while (positionError < static_cast<float>(-M_PI)) {
        positionError += static_cast<float>(M_TWOPI);
    }

    //float choiceKDT = currentDrivetrainVelocity * positionError > 0 ? KDT : KDT_REV;  // check if turret is fighting drivetrain;
    inputVelocity = std::clamp(inputVelocity, -VELO_MAX / 2, VELO_MAX / 2);
    // float targetVelocity = (KP + signum(currentDrivetrainVelocity) * choiceKDT) * positionError + inputVelocity;
    float targetVelocity = decelProfile(positionError, currentVelocity, inputVelocity, currentDrivetrainVelocity);

    // experimental
    //  targetVelocity = signum(positionError)*sqrt(currentVelocity*currentVelocity + 2 * A_DECEL * abs(positionError));

    // model based motion profile
    float aMaxTemp = (C + (KB * KT * RATIO * RATIO) / RA + UK * signum(currentDrivetrainVelocity - currentVelocity));
    float maxVelocity = std::min(VELO_MAX, pastTargetVelocity + 1 / J * (aMaxTemp + (VOLT_MAX * KT * RATIO) / RA) * A_SCALE * deltaT);
    float minVelocity = std::max(-VELO_MAX, pastTargetVelocity + 1 / J * (aMaxTemp - (VOLT_MAX * KT * RATIO) / RA) * A_SCALE * deltaT);
    targetVelocity = std::clamp(targetVelocity, minVelocity, maxVelocity);

    // velocity controller
    float velocityError = targetVelocity - currentVelocity;
    float targetRelativeVelocity = targetVelocity - currentDrivetrainVelocity;
    float targetAcceleration = (targetVelocity - pastTargetVelocity) / deltaT;
    pastTargetVelocity = targetVelocity;

    // integral velocity controller
    if (abs(pastOutput) < INT_THRESH || velocityError * buildup < 0)  // signs are opposite
    {                                                                 // saturation detection
         if (velocityError * buildup < 0) {                              // overshooting
        buildup *= (1 - TAKEBACK);  // take back not quite half
         }
        buildup += velocityError * deltaT;  // integrate normally
    }
    // calculation for setting target current aka velocity controller
    float targetCurrent = std::clamp(KVISC * targetRelativeVelocity + UK * signum(targetRelativeVelocity) + KA * targetAcceleration + KPV * velocityError + KIV * buildup, -20.0f, 20.0f);

    pastOutput = RA * targetCurrent + KV * targetRelativeVelocity;
    pastTorque = targetCurrent*KT;

#if defined(OLDINFANTRY)
    return std::clamp(pastOutput, -VOLT_MAX, VOLT_MAX);
#else
    return 0.8192f * std::clamp(targetCurrent, -20.0f, 20.0f);
#endif
}

void YawController::estimateState(float* theta, float* thetadot, float tLast, float drivetrainVelocity) {
    // For each historical value of Fx, Fy, Tz, calculate the estimates
    for (int i = Q_SIZE - 1; i >= 0; i--) {
        // // Store the current values in the history
        // load in new value if i is not > 0
        // forcehistory[Q_SIZE-1] gets kicked out every method call
        torqueHistory[i] = (i > 0) ? torqueHistory[i - 1] : tLast;

        // Estimated angular velocity (theta_dot)
        *thetadot += (torqueHistory[i] - C*(*thetadot - drivetrainVelocity) - signum(*thetadot - drivetrainVelocity)) * (DT/ J);

        // Update velocity
        *theta += *thetadot * DT;
    }
}

float YawController::decelProfile(float poserror, float thetadot, float thetadotinput, float drivetrainVelocity) {
    float o = 0, o2 = 0;  // offsets
    float t2 = thetadotinput * thetadotinput;
    float v1 = 0, v2 = 0, v3 = 0, v4 = 0;

    o = (thetadotinput - THETA_DOT_BREAK) / KP + (THETA_DOT_BREAK * THETA_DOT_BREAK - t2) / (2 * A_DECEL);
    o2 = (thetadotinput + THETA_DOT_BREAK) / KP + (t2 - THETA_DOT_BREAK * THETA_DOT_BREAK) / (2 * A_DECEL);

    if (t2 - 2 * A_DECEL * (-poserror - o) >= 0) {
        v1 = std::sqrt(t2 - 2 * A_DECEL * (-poserror - o));   // Positive left
        v2 = -std::sqrt(t2 - 2 * A_DECEL * (-poserror - o));  // Negative left
    }
    if (t2 - 2 * A_DECEL * (poserror + o2) >= 0) {
        v3 = std::sqrt(t2 - 2 * A_DECEL * (poserror + o2));   // Positive right
        v4 = -std::sqrt(t2 - 2 * A_DECEL * (poserror + o2));  // Negative right
    }
    if (std::fabs(poserror) < THETA_DOT_BREAK / KP)  // std::fabs(thetadot - thetadotinput) < THETA_DOT_BREAK)
        return (KP + (drivetrainVelocity * poserror > 0 ? KDT : KDT_REV)*drivetrainVelocity) * poserror + thetadotinput;

    if (v3 != 0 && poserror > 0 && thetadot <= 0)
        return v3;
    else if (v2 != 0 && poserror < 0 && thetadot >= 0)
        return v2;
    else if (poserror > 0)
        return v1;
    else
        return v4;
}
}  // namespace subsystems