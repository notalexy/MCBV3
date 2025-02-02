#pragma once
#include <cmath>
#include <iostream>
#include <string>
#include <deque>  // For storing the last n values

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"


namespace subsystems
{
class ChassisController
{

private:
    // START getters and setters
    const float TRACKWIDTH = 0.3;          // in m. We need to measure
    const float ROOT_2 = sqrt(2);          // sqrt 2
    const float M = 14.0;                  // robot mass kg
    const float J = 0;                     // measured from sys id kg-m^2
    const float R_WHEEL = 0.048 / ROOT_2;  // wheel radius m
    const float J_WHEEL = 0;               // wheel moment of inertia kg-m^2
    const float C_WHEEL = 0;               // wheel damping kg-s/m^2
    const float UK_WHEEL = 0;              // wheel dry friction N-m
    const float COF_WHEEL = 0;             // unitless COF

    const float M_EFFECTIVE = M + 4 * J_WHEEL / (R_WHEEL * R_WHEEL);
    const float J_EFFECTIVE = J + 4 * J_WHEEL * TRACKWIDTH / (2 * R_WHEEL);

    const float KB = 0;          // V-s/rad backemf
    const float KT = 0;          // N-m/A torque constant
    const float RA = 0;          // ohm, armature resistance
    const float GEAR_RATIO = 0;  // gear ratio
    const float VOLT_MAX = 0;    // V, maximum voltage
    const float P_MAX = 0;       // W, maximum power
    const float I_MAX = 0;       // A, maximum current

    // Feedforward gains from fundamental system constants
    const float K_V = KB;                   // Velocity feedforward gain (back EMF constant)
    const float K_VIS = C_WHEEL / KT;       // Viscous damping feedforward gain
    const float K_S = UK_WHEEL / KT;        // Static friction feedforward g

    // Tunable Parameters
    const float F_MIN_T = 0;  // minimum beyblade force if throttling
    const float KP_V = 0;     // proportional gain for velocity
    const float KI_V = 0;     // integral gain for velocity

    const float IV_MAX = 0;  // maximum integral term for velocity control

    const float KP_V_ROT = 0;  // proportional gain for rotational velocity

    const float KP = 0;              // proportional gain for position control
    const float VEL_TARGET = 0;      // target velocity
    const float BEYBLADE_DELAY = 0;  // delay for beyblade mode

    const float BEYBLADE_AMPLITUDE = 0;  // beyblade amplitude
    const float BEYBLADE_FREQUENCY = 0;  // beyblade frequency

    const float LATENCY = 0.008;            //latency s
    const float DT = 0.002;                 //DT in s


    float theta_estimated = 0;
    float dot_theta_estimated = 0;
    float dot_theta_estimated_last = 0;
    float estimated_inertial[2] = {0,0}; //x and y
    float dot_estimated_inertial[2] = {0,0}; //x and y
    float dot_estimated_inertial_last[2] = {0,0}; //x and y


    float inertial_forces[2] = {0,0};
    float dot_local[2] = {0,0};

    std::deque<float*> history;
    std::deque<float> target_velocity_queue; // For storing target velocity magnitudes

    // Beyblade settings
    float dot_theta_beyblade = 0;  // Target beyblade velocity when chassis is not translating
    float dot_theta_gain = 0;      // Amount to subtract from dot_theta_beyblade per meter/s of chassis speed

    float F_last_inertial[2] = {0,0}; //x and y
    float Eint_input_local[2] = {0,0}; //x and y
    float E_input_inertial[2] = {0,0}; //x and y

    void forceSummation(float f1, float f2, float f3, float f4, float* fx, float* fy, float* tz)
    {
        *fx = (f1 - f2 - f3 + f4) / ROOT_2;
        *fy = (-f1 - f2 + f3 + f4) / ROOT_2;
        *tz = (-f1 - f2 - f3 - f4) * TRACKWIDTH / 2;
    }
    

    void motorVelocites(float t1m, float t2m, float t3m, float t4m, float* xdot, float* ydot, float* thetadot)
    {
        // t1m is theta 1m dot
        *xdot = (t1m - t2m - t3m + t4m) * GEAR_RATIO / (R_WHEEL * ROOT_2);
        *ydot = (t1m + t2m - t3m - t4m) * GEAR_RATIO / (R_WHEEL * ROOT_2);
        *thetadot = (-t1m - t2m - t3m - t4m) * GEAR_RATIO * TRACKWIDTH / (R_WHEEL * 2);
    }

    //initialize the rest in the function
    float** rotMat = new float*[2]; 
    
    // rotation matrix function
    float** rotationMatrix(float theta) {
        rotMat[0][0] = std::cos(theta);
        rotMat[1][0] = std::sin(theta);
        rotMat[0][1] = -rotMat[1][0];
        rotMat[1][1] = rotMat[0][0];
        return rotMat;
    }
    void multiplyMatrices(int rows1, int cols1, float** mat1, float* mat2, float* result) {
        // Iterate over rows of mat1 and columns of mat2
        for (int i = 0; i < rows1; ++i) {
            result[i] = 0;  // Initialize to zero before accumulating
            for (int j = 0; j < cols1; ++j) {
                result[i] += mat1[i][j] * mat2[j];
            }
        }
    }


    void estimateState(float* F);

    void estimateInputError();

    void calculateRequiredForces();



    float signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }

        // Helper for velocity control (sawtooth function)
    float sawtooth(float freq, float amplitude)
    {
        // A simple sawtooth wave function for variable speed beyblade
        return amplitude * (1 - fmod(freq * (std::fmod(std::clock(), 1.0f) * 2 * M_PI), 1.0f));
    }


public:
    ChassisController();
    //~YawController();
    float calculate();
    float calculateBeybladeVelocity(float bb_freq, float bb_amp);

};
}  // namespace subsystems
