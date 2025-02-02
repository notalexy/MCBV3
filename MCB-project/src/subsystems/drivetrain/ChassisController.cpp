#include "ChassisController.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

namespace subsystems
{
ChassisController::ChassisController() {
    rotMat[0] = new float[2];
    rotMat[1] = new float[2];
}

float ChassisController::calculateBeybladeVelocity(float bb_freq, float bb_amp) {
        // Get the target velocity commands (or position commands) from the controller
        // This part would integrate the user input or other controller logic to set target velocity

        float velmag_max = 0;
        for (const auto& vel : target_velocity_queue) {
            velmag_max = std::max(velmag_max, vel); // Find max velocity magnitude in the last bb_delay seconds
        }

        // If fixed-speed beyblade or variable-speed beyblade with velocity != 0
             // Variable-speed beyblade behavior when velmag_max == 0
        if (velmag_max != 0)  dot_theta_beyblade -= velmag_max * dot_theta_gain;
        else if (velmag_max == 0 && bb_amp != 0)  dot_theta_beyblade -= (bb_amp / 2) + sawtooth(bb_freq, bb_amp);
        else dot_theta_beyblade = 0;

        // Update target velocity queue
        target_velocity_queue.push_back(VEL_TARGET);
        if (target_velocity_queue.size() > BEYBLADE_DELAY / DT) {
            target_velocity_queue.pop_front();
        }

        return dot_theta_beyblade;  // Return the required angular velocity (dot_theta_req)
    }


 // Function to calculate chassis state using historical force/torque data and latency
    void ChassisController::estimateState(float* F) {
        // Store the current values in the history
        history.push_back(F);

        // If the queue exceeds size n (based on latency), remove the oldest values
        if (history.size() > LATENCY/DT) {
            history.pop_front();
        }

        // For each historical value of Fx, Fy, Tz, calculate the estimates
        for (int i = 0; i < LATENCY/DT; ++i) {
            // Estimated angular velocity (theta_dot)
            dot_theta_estimated += history[i][2] * DT / J_EFFECTIVE;

            // Update theta estimate (added DT because i think alex Y did it wrong)
            theta_estimated += dot_theta_estimated * DT;

            // Local to inertial frame transformation
            multiplyMatrices(2, 2, rotationMatrix(theta_estimated), history[i], inertial_forces);


            // Update position estimates
            dot_estimated_inertial[0] += inertial_forces[0] * DT / M_EFFECTIVE;
            dot_estimated_inertial[1] += inertial_forces[1] * DT / M_EFFECTIVE;


            estimated_inertial[0] += dot_estimated_inertial[0] * DT;
            estimated_inertial[1] += dot_estimated_inertial[1] * DT;

        }

        // Convert back to local frame (if necessary)
        multiplyMatrices(2, 2, rotationMatrix(-theta_estimated), dot_estimated_inertial, dot_local);
        
    }
    // Function to estimate input errors in inertial frame
void ChassisController::estimateInputError() {
    for(int i = 0; i < 1; i++){ //iterate for x and y axes
        //i removed fest, because it is stored as inertial forces
        
        // Compute the error between the last and the estimated forces in the inertial frame
        E_input_inertial[i] = F_last_inertial[i] - inertial_forces[i];

        // Update integral error terms
        Eint_input_local[i] = KI_V * (E_input_inertial[i] - Eint_input_local[i]);

        // Store current forces for the next iteration
        F_last_inertial[i] = inertial_forces[i];

    }
}

// Function to calculate the required force (and torque) based on errors
void ChassisController::calculateRequiredForces() {
    // Calculate required forces for X and Y in local frame
    float F_x_req_local = KP_V * (dot_estimated_inertial[0] - dot_estimated_inertial_last[0]) + Eint_input_local[0];
    float F_y_req_local = KP_V * (dot_estimated_inertial[1] - dot_estimated_inertial_last[1]) + Eint_input_local[1];
    
    // Calculate required torque for Z in local frame
    float T_z_req_local = KP_V_ROT * (dot_theta_estimated - dot_theta_estimated_last);

    // Update the estimated velocities for the next iteration
    dot_estimated_inertial_last[0] = dot_estimated_inertial[0];
    dot_estimated_inertial_last[1] = dot_estimated_inertial[1];
    dot_theta_estimated_last = dot_theta_estimated;
    
    // Here, you'd apply the forces and torque to your motors or control systems
}
float ChassisController::calculate() {
    // First, estimate the input errors
    estimateInputError();

    // Then, calculate the required forces
    calculateRequiredForces();

    // Return the calculated force (or any other desired output)
    return 0; // Placeholder, return the required output as needed
}



}  // namespace ThornBots
