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
ChassisController::ChassisController() {
    rotMat[0] = new float[2];
    rotMat[1] = new float[2];
}

float ChassisController::calculateBeybladeVelocity(float bb_freq, float bb_amp) {
        // Get the target velocity commands (or position commands) from the controller
        // This part would integrate the user input or other controller logic to set target velocity

        float velmagMax = 0;
        for (const auto& vel : targetVelocityQueue) {
            velmagMax = std::max(velmagMax, vel); // Find max velocity magnitude in the last bb_delay seconds
        }

        // If fixed-speed beyblade or variable-speed beyblade with velocity != 0
             // Variable-speed beyblade behavior when velmagMax == 0
        if (BEYBLADE_FIXED_SPEED || velmagMax != 0)  dotThetaBeyblade -= velmagMax * dotThetaGain;
        else if (!BEYBLADE_FIXED_SPEED && velmagMax == 0)  dotThetaBeyblade -= (bb_amp / 2) + sawtooth(bb_freq, bb_amp);
        else dotThetaBeyblade = 0;

        // Update target velocity queue
        targetVelocityQueue.emplace_back(VEL_TARGET);
        if (targetVelocityQueue.size() > BEYBLADE_DELAY / DT) {
            targetVelocityQueue.pop_front();
        }

        return dotThetaBeyblade;  // Return the required angular velocity (dot_theta_req)
    }


 // Function to calculate chassis state using historical force/torque data and latency
    void ChassisController::estimateState(float* F) {
        int n = static_cast<int32_t>(LATENCY/DT); // Should be good but casting it just in case
        // Store the current values in the history
        history.emplace_back(F);

        // If the queue exceeds size n (based on latency), remove the oldest values by resizing it down to n
        history.resize(n);

        // For each historical value of Fx, Fy, Tz, calculate the estimates
        for (int i = 0; i < n; ++i) {
            // Estimated angular velocity (theta_dot)
            dotThetaEstimated += (history[i][2] * DT) / J_EFFECTIVE;

            // Update theta estimate (added DT because i think alex Y did it wrong)
            thetaEstimated += dotThetaEstimated * DT;

            // Local to inertial frame transformation
            multiplyMatrices(2, 2, rotationMatrix(thetaEstimated), history[i], inertialForces);


            // Update position estimates
            dotEstimatedInertial[0] += inertialForces[0] * DT / M_EFFECTIVE;
            dotEstimatedInertial[1] += inertialForces[1] * DT / M_EFFECTIVE;


            estimatedInertial[0] += dotEstimatedInertial[0] * DT;
            estimatedInertial[1] += dotEstimatedInertial[1] * DT;

        }

        // Convert back to local frame (if necessary)
        multiplyMatrices(2, 2, rotationMatrix(-thetaEstimated), dotEstimatedInertial, dotLocal);
        
    }
    // Function to estimate input errors in inertial frame
    void ChassisController::estimateInputError() {
        for(int i = 0; i < 2; i++){ //iterate for x and y axes
            //i removed fest, because it is stored as inertial forces
            
            // Compute the error between the last and the estimated forces in the inertial frame
            E_inputInertial[i] = F_lastInertial[i] - inertialForces[i];

            // Update integral error terms
            Eint_inputLocal[i] = KI_V * (E_inputInertial[i] - Eint_inputLocal[i]);

            // Store current forces for the next iteration
            F_lastInertial[i] = inertialForces[i];

        }
    }

    // Function to calculate the required force (and torque) based on errors
    void ChassisController::calculateRequiredForces() {
        // Calculate required forces for X and Y in local frame
        F_x_reqLocal = KP_V * (dotEstimatedInertial[0] - dotEstimatedInertialLast[0]) + Eint_inputLocal[0];
        F_y_reqLocal = KP_V * (dotEstimatedInertial[1] - dotEstimatedInertialLast[1]) + Eint_inputLocal[1];
        
        // Calculate required torque for Z in local frame
        T_z_reqLocal = KP_V_ROT * (dotThetaEstimated - dotThetaEstimatedLast);

        // Update the estimated velocities for the next iteration
        dotEstimatedInertialLast[0] = dotEstimatedInertial[0];
        dotEstimatedInertialLast[1] = dotEstimatedInertial[1];
        dotThetaEstimatedLast = dotThetaEstimated;
        
    }

    // Follows the modeled drivetrain velocity on the notion
    void ChassisController::estimateInverseKinematics()
    {
        // Get the vector and the matrix
        float vec[3] = {dotEstimatedInertial[0], dotEstimatedInertial[1], thetaEstimated};
        float *matrix[4];
        float row1[3] = {1, 1, (-TRACKWIDTH * ROOT_2) / 2};
        float row2[3] = {-1, 1, (-TRACKWIDTH * ROOT_2) / 2};
        float row3[3] = {-1, -1, (-TRACKWIDTH * ROOT_2) / 2};
        float row4[3] = {1, -1, (-TRACKWIDTH * ROOT_2) / 2};
        matrix[0] = row1;
        matrix[1] = row2;
        matrix[2] = row3;
        matrix[3] = row4;
        // Multiply them
        multiplyMatrices(4, 3, matrix, vec, estimatedMotorTheta);

        // Then scale the resulting estimated motor theta by the constant in Modeling
        for (int i = 0; i < 4; ++i)
        {
            estimatedMotorTheta[i] *= GEAR_RATIO / (R_WHEEL * ROOT_2);
        }
    }

    // Get the feed forward variables for each motor based on the previously calculaed inverse kinematic estimated theta (see feedforwards)
    void ChassisController::calculateFeedForward()
    {
        float estimatedTheta;
        for (int i = 0; i < 4; ++i)
        {
            estimatedTheta = estimatedMotorTheta[i];
            motor_V_I[i].first = K_V * estimatedTheta;
            motor_V_I[i].second = K_VIS * estimatedTheta + K_S * signum(estimatedTheta);
        }
    }

    void ChassisController::calculateTractionLimiting()
    {
        float beybladeCommand = calculateBeybladeVelocity(0.0f, 0.0f); // not sure what I should pass into here
        // Manipulate F_largest based on whether the beyblade torque command is positive or negative
        if (beybladeCommand > 0)
        {
            // TODO: ask how to get minimum/maximum output torque
        }
        else if (beybladeCommand < 0)
        {

        }

        float F_too_much = std::min(abs(F_largest) - F_MAX / 4, 0.0f); // clamp between 0 and infinity

        float T_req_throttled = signum(T_z_reqLocal) * std::min(abs(T_z_reqLocal), 2 * TRACKWIDTH * std::max(T_z_reqLocal / (2 * TRACKWIDTH) - F_too_much, F_MIN_T));

        // Clamping but for dummies. Exclusive from 0 so this has to be done
        if (F_lat_wheel_max == 0)
        {
            lateralScalingFactor = 1;
        }
        else if (F_lat_wheel_max == INFINITY)
        {
            lateralScalingFactor = 0;
        }
        else
        {
            lateralScalingFactor = std::min(std::max((F_MAX / 4 - T_req_throttled / (2 * TRACKWIDTH)) - 1 / F_lat_wheel_max, 1.0f), 0.0f);
        }

        F_req_throttled[0] = lateralScalingFactor * F_x_reqLocal; // x
        F_req_throttled[1] =  lateralScalingFactor * F_y_reqLocal; // y
    }

    // Corresponds to the power limiting Notion file
    void ChassisController::calculateTReqM()
    {
        float vec[3] = {F_req_throttled[0], (F_req_throttled[1], 2 * T_z_reqLocal) / (TRACKWIDTH * ROOT_2)};
        float *matrix[4];
        float row1[3] = {1, 1, -1};
        float row2[3] = {-1, 1, -1};
        float row3[3] = {-1, -1, -1};
        float row4[3] = {1, -1, -1};
        matrix[0] = row1;
        matrix[1] = row2;
        matrix[2] = row3;
        matrix[3] = row4;
        // Multiply them
        multiplyMatrices(4, 3, matrix, vec, T_req_m);

        // Scale it at the end
        for (int i = 0; i < 4; ++i)
        {
            T_req_m[i] *= (R_WHEEL * ROOT_2) / (4 * (GEAR_RATIO));
        }

    }

    // Calculates scalding factor based on the equations in the notion
    void ChassisController::calculatePowerLimiting()
    {

        // Get all the summations out of the way first
        float aSum = 0, bSumFirst = 0, bSumSecond = 0, cSumFirst = 0, cSumSecond = 0;
        for (int i = 0; i < 4; ++i)
        {
            aSum += T_req_m[i] * T_req_m[i];
            bSumFirst += motor_V_I[i].second * KT * T_req_m[i];
            bSumSecond += T_req_m[i] * motor_V_I[i].first;
            cSumFirst += motor_V_I[i].second * motor_V_I[i].second;
            cSumSecond += motor_V_I[i].second * KT * motor_V_I[i].first;
        }
        
        // Then get a, b, c to calculate the scalding factor with
        float a = (RA / (KT * KT)) * aSum;
        float b = (2 * RA) / (KT * KT) * bSumFirst + (1 / KT) * bSumSecond;
        float c =  (RA / (KT * KT)) * cSumFirst + (1 / KT) * cSumSecond - P_MAX;

        scaldingFactor = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);

        //constrain to 0 and 1
        scaldingFactor = std::max(std::min(scaldingFactor, 1.0f), 0.0f);

        // And finally get T_req_m_throttled based on the new scalding factor
        for (int i = 0; i < 4; ++i)
        {
            T_req_m_throttled[i] = scaldingFactor * T_req_m[i];
        }
    }

    void ChassisController::calculateMotorSummations()
    {
        for (int i = 0; i < 4; ++i)
        {
            motorCurrent[i] = KT / T_req_m_throttled[i] + motor_V_I[i].second;
        }
    }

    float ChassisController::calculate()
    {
        estimateInverseKinematics();

        estimateState(nullptr);

        calculateBeybladeVelocity(0.0f, 0.0f); // this should have a class variable linked to the return so traction limiting can use it

        calculateFeedForward();
        // First, estimate the input errors
        estimateInputError();

        // Then, calculate the required forces
        calculateRequiredForces();

        calculateTractionLimiting();

        calculateTReqM();

        calculatePowerLimiting();

        calculateMotorSummations();

        // Return the calculated force (or any other desired output)
        return 0; // Placeholder, return the required output as needed
    }

}  // namespace ThornBots
