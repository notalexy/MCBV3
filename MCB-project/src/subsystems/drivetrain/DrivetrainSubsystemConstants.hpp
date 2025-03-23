#pragma once
#include "tap/algorithms/smooth_pid.hpp"
namespace subsystems {

// START getters and setters
#if defined(HERO)
constexpr float KP_ROT = 0.5;  // proportional gain for drivetrain 
constexpr float KI_ROT = 0;      // integral gain for velocity
constexpr float KD_ROT = 0.5;      // derivative gain for drivetrain
constexpr float MAX_ISUM_ROT = 0;  // maximum integral sum term for drivetrain
constexpr float MAX_OUTPUT_ROT = 2.0;  // maximum output for drivetrain rotation controller

constexpr float ERR_DEADZONE_ROT = 0.05;  // error deadzone for drivetrain rad/s

constexpr float PEEK_LEFT_AMT = 0.2;  // amount to peek left
constexpr float PEEK_RIGHT_AMT = -0.2;  // amount to peek right
#elif defined(SENTRY)
constexpr float KP_ROT = 0.5;  // proportional gain for drivetrain 
constexpr float KI_ROT = 0;      // integral gain for velocity
constexpr float KD_ROT = 0.5;      // derivative gain for drivetrain
constexpr float MAX_ISUM_ROT = 0;  // maximum integral sum term for drivetrain
constexpr float MAX_OUTPUT_ROT = 2.0;  // maximum output for drivetrain rotation controller

constexpr float ERR_DEADZONE_ROT = 0.05;  // error deadzone for drivetrain rad/s

constexpr float PEEK_LEFT_AMT = 0.2;  // amount to peek left
constexpr float PEEK_RIGHT_AMT = -0.2;  // amount to peek right
#elif defined(INFANTRY)
constexpr float KP_ROT = 0.5;  // proportional gain for drivetrain 
constexpr float KI_ROT = 0;      // integral gain for velocity
constexpr float KD_ROT = 0.5;      // derivative gain for drivetrain
constexpr float MAX_ISUM_ROT = 0;  // maximum integral sum term for drivetrain
constexpr float MAX_OUTPUT_ROT = 2.0;  // maximum output for drivetrain rotation controller

constexpr float ERR_DEADZONE_ROT = 0.05;  // error deadzone for drivetrain rad/s

constexpr float PEEK_LEFT_AMT = 0.2;  // amount to peek left
constexpr float PEEK_RIGHT_AMT = -0.2;  // amount to peek right
#else
constexpr float KP_ROT = 0.5;  // proportional gain for drivetrain 
constexpr float KI_ROT = 0;      // integral gain for velocity
constexpr float KD_ROT = 0.5;      // derivative gain for drivetrain
constexpr float MAX_ISUM_ROT = 0;  // maximum integral sum term for drivetrain
constexpr float MAX_OUTPUT_ROT = 2.0;  // maximum output for drivetrain rotation controller

constexpr float ERR_DEADZONE_ROT = 0.05;  // error deadzone for drivetrain rad/s

constexpr float PEEK_LEFT_AMT = 0.2;  // amount to peek left
constexpr float PEEK_RIGHT_AMT = -0.2;  // amount to peek right
#endif

// after the ifdefs

constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{KP_ROT, KI_ROT, KD_ROT, MAX_ISUM_ROT, MAX_OUTPUT_ROT, 0.0f, 0.0f, 0.0f, 0.0f, ERR_DEADZONE_ROT};
constexpr float DEFAULT_POWER_LIMIT = 100;  // default power limit for drivetrain


}  // namespace subsystems
