#include "YawController.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "YawControllerConstants.hpp"

namespace subsystems
{
YawController::YawController() {}

float YawController::calculate(float currentPosition, float currentVelocity, float currentDrivetrainVelocity, float targetPosition, float inputVelocity, float deltaT)
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

    float choiceKDT = currentDrivetrainVelocity * positionError > 0 ? KDT : KDT_REV;  // check if turret is fighting drivetrain;
    inputVelocity = std::clamp(inputVelocity, -VELO_MAX / 2, VELO_MAX / 2);
    // float targetVelocity = (KP + signum(currentDrivetrainVelocity) * choiceKDT) * positionError + inputVelocity;
    float targetVelocity = decelProfile(positionError, currentVelocity, inputVelocity);
    //experimental
    // targetVelocity = signum(positionError)*sqrt(currentVelocity*currentVelocity + 2 * A_DECEL * abs(positionError));

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
        // if (velocityError * buildup < 0) {                              // overshooting
        buildup *= (1 - TAKEBACK);  // take back not quite half
        // }    
        buildup += velocityError * deltaT;  // integrate normally
    }
    // calculation for setting target current aka velocity controller
    float targetCurrent = KVISC * targetRelativeVelocity + UK * signum(targetRelativeVelocity) + KA * targetAcceleration + KPV * velocityError + KIV * buildup;

    pastOutput = RA * targetCurrent + KV * targetRelativeVelocity;
    return std::clamp(pastOutput, -VOLT_MAX, VOLT_MAX);
}

float YawController::decelProfile(float poserror, float thetadot, float thetadotinput){
    float o = 0, o2 = 0; //offsets
    float t2 = thetadotinput * thetadotinput;
    float v1 = 0, v2 = 0, v3 = 0, v4 = 0;
    // if(std::fabs(thetadotinput) > THETA_DOT_BREAK) {
        // o = (thetadotinput - thetadotbreak)/kp + (thetadotbreak*thetadotbreak - t2) / (2*adecel);
        // o2 = (thetadotinput + thetadotbreak)/kp + (t2 - thetadotbreak*thetadotbreak) / (2*adecel);
    // }
    if (t2 - 2*A_DECEL*(-poserror - o) >= 0) {
        v1 = std::sqrt(t2 - 2*A_DECEL*(-poserror - o)); //Positive left
        v2 = -std::sqrt(t2 - 2*A_DECEL*(-poserror - o)); //Negative left
    }
    if (t2 - 2*A_DECEL*(poserror + o2) >= 0) {
        v3 = sqrt(t2 - 2*A_DECEL*(poserror + o2)); //Positive right
        v4 = -sqrt(t2 - 2*A_DECEL*(poserror + o2)); //Negative right
    }
    if (std::fabs(thetadot - thetadotinput) < THETA_DOT_BREAK)
        return KP*poserror + thetadotinput;
    else if (v3 != 0 && poserror > 0 && thetadot <= 0)
        return v3;
    else if (v2 != 0 && poserror < 0 && thetadot >= 0)
        return v2;
    else if (thetadot > 0 || thetadot == 0 && poserror > 0)
        return v1;
    else
        return v4;
}

}  // namespace subsystems
