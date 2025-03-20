#include "tap/algorithms/smooth_pid.hpp"

namespace subsystems {

constexpr static int INDEXER_MOTOR_MAX_SPEED = 6177; // With the 2006, this should give
constexpr static float REV_PER_BALL = 36.0f / 7.0f; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};

}