#include "ChassisController.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

// TODO: Calculate function that takes in target x, y velocities as well as a target angle as inputs, outputs 4 motor current summations

namespace subsystems
{
ChassisController::ChassisController() {}

// Function to calculate chassis state using historical force/torque data and latency
void ChassisController::estimateState(Pose2d &inputForces, Pose2d &eLocalVel, Pose2d &eInertialVel, Pose2d &eInertialPos)
{
    int n = static_cast<int32_t>(LATENCY / DT);  // Should be good but casting it just in case
    // Store the current values in the history
    history.emplace_back(&inputForces);

    // If the queue exceeds size n (based on latency), remove the oldest values by resizing it down to n
    history.resize(n);

    // For each historical value of Fx, Fy, Tz, calculate the estimates
    for (int i = 0; i < n; ++i)
    {
        // Estimated angular velocity (theta_dot)
        eInertialVel.rotation += history[i]->rotation * DT / J_EFFECTIVE;

        // Update theta estimate (added DT because i think alex Y did it wrong)
        eInertialPos.rotation += eInertialVel.rotation * DT;

        // Update velocity
        eInertialVel = eInertialVel.addXY(history[i]->rotate(-eInertialPos.rotation).scalarMultiply(DT / M_EFFECTIVE));

        // update positions
        eInertialPos = eInertialPos.addXY(eInertialVel.scalarMultiply(DT));
    }

    eLocalVel = eInertialVel.rotate(eInertialPos.rotation);
}

float ChassisController::calculateBeybladeVelocity(float bb_freq, float bb_amp)
{
    // Get the target velocity commands (or position commands) from the controller
    // This part would integrate the user input or other controller logic to set target velocity

    float velmagMax = 0;
    for (float vel : targetVelocityQueue)
    {
        velmagMax = std::max(velmagMax, vel);  // Find max velocity magnitude in the last bb_delay seconds
    }

    // If fixed-speed beyblade or variable-speed beyblade with velocity != 0
    // Variable-speed beyblade behavior when velmagMax == 0
    if (BEYBLADE_FIXED_SPEED || velmagMax != 0)
        dotThetaBeyblade -= velmagMax * dotThetaGain;
    else if (!BEYBLADE_FIXED_SPEED && velmagMax == 0)
        dotThetaBeyblade -= (bb_amp / 2) + sawtooth(bb_freq, bb_amp);
    else
        dotThetaBeyblade = 0;

    // Update target velocity queue
    targetVelocityQueue.emplace_back(VEL_TARGET);
    if (targetVelocityQueue.size() > BEYBLADE_DELAY / DT)
    {
        targetVelocityQueue.pop_front();
    }

    return dotThetaBeyblade;  // Return the required angular velocity (dot_theta_req)
}

Pose2d lastInertialVel = {0, 0, 0}, accumLocalForce = {0, 0, 0};
// Function to estimate input errors in inertial frame
void ChassisController::velocityControl(Pose2d &inputLocalVel, Pose2d &estInertialVel, Pose2d &estLocalVel, Pose2d &lastInertialForce, Pose2d &reqLocalForce)
{
    Pose2d estInertialForce = estInertialVel.subXY(lastInertialVel).scalarMultiply(M_EFFECTIVE / DT);

    // Compute the error between the last and the estimated forces in the inertial frame
    Pose2d errorInertialForce = lastInertialForce.subXY(estInertialForce);

    // Update integral error terms
    accumLocalForce = accumLocalForce.addXY(errorInertialForce.subXY(accumLocalForce).scalarMultiply(KI_V * DT));

    // update the required local force
    reqLocalForce = inputLocalVel.subXY(estLocalVel).scalarMultiply(KP_V).addXY(accumLocalForce);

    // Store current forces for the next iteration
    lastInertialVel = estInertialVel.copy();
}

// Get the feed forward variables for each motor based on the previously calculaed inverse kinematic estimated theta (see feedforwards)
void ChassisController::calculateFeedForward(std::vector<float> &estimatedMotorVelocity, std::vector<float> &V_m_FF, std::vector<float> &I_m_FF)
{
    float vel;
    for (int i = 0; i < 4; ++i)
    {
        vel = estimatedMotorVelocity[i];
        V_m_FF[i] = K_V * vel;
        I_m_FF[i] = K_VIS * vel + K_S * signum(vel);
    }
}

void ChassisController::calculateTractionLimiting(Pose2d &localForce)
{
    float beybladeCommand = 0;//calculateBeybladeVelocity(0.0f, 0.0f);  // not sure what I should pass into here

    std::vector<float> motorTorque(4, 0.0f);

    multiplyMatrices(4, 3, forceInverseKinematics, localForce.toVector(), motorTorque);

    float largest = motorTorque[0];

    // Manipulate F_largest based on whether the beyblade torque command is positive or negative
    for (int i = 1; i < 4; i++)
    {  // if beybladeCommand > 0, find smallest, vice versa
        if (beybladeCommand > 0 ^ motorTorque[i] > largest)
        {
            largest = motorTorque[i];
        }
    }

    float F_too_much = std::max(abs(largest) - F_MAX / 4, 0.0f);  // clamp between 0 and infinity

    float T_req = localForce.rotation;

    float T_req_throttled = signum(T_req) * std::min(abs(T_req), 2 * TRACKWIDTH * std::max(T_req / (2 * TRACKWIDTH) - F_too_much, F_MIN_T));

    std::vector<float> F_lat(4, 0.0f);

    // 2 rows to not do torque
    multiplyMatrices(4, 2, forceInverseKinematics, localForce.toVector(), F_lat);

    largest = abs(F_lat[0]);

    // Manipulate F_largest based on whether the beyblade torque command is positive or negative
    for (int i = 1; i < 4; i++)
    {  // if beybladeCommand > 0, find smallest, vice versa
        if (abs(F_lat[i]) > largest)
        {
            largest = abs(F_lat[i]);
        }
    }
    // since 1/0 is inf, 1/0.00000000000001 is inf basically, do nothing
    if (largest == 0) return;

   localForce = localForce.scalarMultiply(std::clamp((F_MAX / 4 - T_req_throttled / (2 * TRACKWIDTH)) * 1 / largest, 0.0f, 1.0f));
}

// Calculates scaling factor based on the equations in the notion
void ChassisController::calculatePowerLimiting(Pose2d &localForce, std::vector<float> &V_m_FF, std::vector<float> &I_m_FF, std::vector<float> &T_req_m)
{
    multiplyMatrices(4, 3, forceInverseKinematics, localForce.scalarMultiply(R_WHEEL / GEAR_RATIO).toVector(), T_req_m);

    // // Get all the summations out of the way first
    float aSum = 0, bSumFirst = 0, bSumSecond = 0, cSumFirst = 0, cSumSecond = 0;
    for (int i = 0; i < 4; i++)
    {
        aSum += T_req_m[i] * T_req_m[i];
        bSumFirst += I_m_FF[i] * T_req_m[i];
        bSumSecond += V_m_FF[i] * T_req_m[i];
        cSumFirst += I_m_FF[i] * I_m_FF[i];
        cSumSecond += I_m_FF[i] * V_m_FF[i];
    }

    // Then get a, b, c to calculate the scaling factor with
    float a = (RA / (KT * KT)) * aSum;                              // KT^-2 T^2
    float b = ((2 * RA) / KT) * bSumFirst + (1 / KT) * bSumSecond;  // KT^-1 T^1 IV^1
    float c = RA * cSumFirst + cSumSecond - P_MAX;                  // IV^2

    // constrain to 0 and 1
    float s_scaling = std::clamp((-b + sqrt(b * b - 4 * a * c)) / (2 * a), 0.0f, 1.0f);

    // And finally get T_req_m_throttled based on the scaling factor
    for (int i = 0; i < 4; ++i)
    {
        T_req_m[i] = s_scaling * T_req_m[i];
    }
}

Pose2d estimatedLocalVelocity, estimatedInertialVelocity, estimatedInertialPosition;

Pose2d currentInertialPosition, currentLocalVelocity;

std::vector<float> V_m_FF, I_m_FF, estimatedMotorVelocity, motorTorque;  // motor feedforwards

Pose2d lastLocalForce, lastInertialForce;

void ChassisController::calculate(Pose2d &targetVelLocal, float &angle, float motorVelocity[4], float motorCurrent[4])
{
    std::vector<float> vec(3, 0.0f);
    std::vector<float> measMotorVelocity(4, 0.0f);

    for(int i = 0; i < 4; i++){
        measMotorVelocity[i] = motorVelocity[i];
    }

    multiplyMatrices(3, 4, forwardKinematics, measMotorVelocity, vec);

    estimatedLocalVelocity = {vec[0], vec[1], vec[2]};

    estimatedInertialPosition = {0, 0, angle};

    estimateState(lastLocalForce, estimatedLocalVelocity, estimatedInertialVelocity, estimatedInertialPosition);

    // inverse kinematics
    multiplyMatrices(4, 3, inverseKinematics, estimatedLocalVelocity.toVector(), estimatedMotorVelocity);

    // calculateBeybladeVelocity(0.0f, 0.0f);  // should be changed

    calculateFeedForward(estimatedMotorVelocity, V_m_FF, I_m_FF);

    // get last inertial force too
    lastInertialForce = lastLocalForce.rotate(estimatedInertialPosition.rotation);

    // // First, estimate the input errors
    velocityControl(targetVelLocal, estimatedInertialVelocity, estimatedLocalVelocity, lastInertialForce, lastLocalForce);

    // motorTorque[0] = lastLocalForce.x;
    calculateTractionLimiting(lastLocalForce);
    // std::vector<float> vec3(3, 0.0f);
  
  
    // vec3[0] = lastLocalForce.x;
    // vec3[1] = lastLocalForce.y;
    // vec3[2] = lastLocalForce.rotation;

    // multiplyMatrices(4, 3, forceInverseKinematics, lastLocalForce.elementMultiply(R_WHEEL/GEAR_RATIO).toVector(), motorTorque);

    // motorTorque[0] = R_WHEEL*ROOT_2/(4*GEAR_RATIO)*(lastLocalForce.x + lastLocalForce.y - lastLocalForce.rotation*2/(TRACKWIDTH*ROOT_2));
    // motorTorque[1] = R_WHEEL*ROOT_2/(4*GEAR_RATIO)*(-lastLocalForce.x + lastLocalForce.y - lastLocalForce.rotation*2/(TRACKWIDTH*ROOT_2));
    // motorTorque[2] = R_WHEEL*ROOT_2/(4*GEAR_RATIO)*(-lastLocalForce.x - lastLocalForce.y - lastLocalForce.rotation*2/(TRACKWIDTH*ROOT_2));
    // motorTorque[3] = R_WHEEL*ROOT_2/(4*GEAR_RATIO)*(lastLocalForce.x - lastLocalForce.y - lastLocalForce.rotation*2/(TRACKWIDTH*ROOT_2));
    

    calculatePowerLimiting(lastLocalForce, V_m_FF, I_m_FF, motorTorque);

    std::vector<float> vec2(3, 0.0f);

    multiplyMatrices(3, 4, forceKinematics, motorTorque, vec2);

    lastLocalForce = {vec2[0], vec2[1], vec2[2]};

    // set motor currents
    // motorCurrent[0] = motorTorque[0];
    for (int i = 0; i < 4; ++i)
    {
        motorCurrent[i] = motorTorque[i] / KT + I_m_FF[i];
    }
}

}  // namespace subsystems
