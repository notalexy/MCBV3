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
class PitchController
{
public:
    PitchController();
    //~PitchController();
    float calculate(
        float currentPosition,
        float currentVelocity,
        float targetPosition,
        float deltaT);

    void clearBuildup() { buildup = 0; };

private:
    // START getters and setters
    float buildup = 0;
    float pastTargetVelocity = 0;
    float pastOutput = 0;
    float targetVelo;
    float currentVelo;
    float targetPos;
    float currentPos;
    float output2;

    // Physical constants
    const float KB = 0.716;                            // V-rad/s
    const float RA = 8.705;                            // ohm
    const float RATIO = 1;                             // unitless
    const float VOLT_MAX = 22.2;                       // V
    const float VELO_MAX = VOLT_MAX / (KB * RATIO);    // rad/s
    const float ACCEL_MAX = 40.0;                      // rad/s

    // Position controller constants
    const float KP = 15;  // sec^-1

    // Feedforward constants
    const float KSTATIC = 0.1;                          // A
    const float KV = KB * RATIO;                        // V-s/rad
    const float KF = 0.05;//-0.001;                     // A

    // Velocity feedback
    const float KPV = 0.5;      // 0.3                  // A-s/rad
    const float KIV = 6;           // 2                 // A/rad
    const float IV_MAX = 0.1;                   // units TBD
    const float INT_THRESH = VOLT_MAX * 0.85l;          // V
    const float TAKEBACK = 0.01;                        // unitless

    int signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }
};
}  // namespace ThornBots
