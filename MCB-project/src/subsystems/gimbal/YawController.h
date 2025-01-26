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
class YawController
{
public:
    YawController();
    //~YawController();
    float calculate(
        float currentPosition,
        float currentVelocity,
        float currentDrivetrainVelocity,
        float targetPosition,
        float inputVelocity,
        float deltaT);

private:
    // START getters and setters
    float buildup = 0;
    float pastTargetVelocity = 0;
    float pastOutput = 0;

    // Physical constants
    const float C = 0.0169;                          // kg-s/m^2
    const float J = 0.031;                           // 289;               // kg-m^2
    const float UK = 0.05;                           // N-m
    const float KB = 0.716;                          // V-rad/s
    const float KT = 0.741;                          // N-m/A
    const float RA = 8.705;                          // ohm
    const float RATIO = 1;                           // unitless
    const float VOLT_MAX = 22.2;                     // V
    const float VELO_MAX = VOLT_MAX / (KB * RATIO);  // rad/s
    // Position controller constants
    const float KP = 11.1;  // sec^-1

    // Feedforward constants
    const float A_SCALE = 0.9;                       // 0.8            // unitless
    const float KSTATIC = (UK * RA) / (KT * RATIO);  // A
    const float KV = KB * RATIO;                     // V-s/rad
    const float KA = J / (KT * RATIO);               // A-s^2/rad
    const float KVISC = C / (KT * RATIO);            // A-s/rad

    // Gain scheduling
    const float KDT = -0.47;     // unitless
    const float KDT_REV = -0.7;  // unitless

    // Velocity feedback
    const float KPV = 1;                        // A-s/rad
    const float KIV = 1.5;//1.5                      // A/rad
    const float IV_MAX = 0.1;                   // units TBD
    const float INT_THRESH = VOLT_MAX * 0.85l;  // V
    const float TAKEBACK = 0.01;                // unitless

    int signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }
};
}  // namespace ThornBots
