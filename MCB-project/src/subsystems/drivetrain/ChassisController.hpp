#pragma once
#include <cmath>
#include <iostream>
#include <string>

#include "util/Pose2d.hpp"

namespace subsystems {
// REMEMBER THAT YOU USE NEGATIVE FOR INERTIAL TO LOCAL AND POSITIVE FOR LOCAL TO INERTIAL
class ChassisController {
private:
    // START getters and setters
    const float TRACKWIDTH = 0.5017;       // in m. We need to measure
    const float ROOT_2 = sqrt(2);          // sqrt 2
    const float M = 14.0;                  // robot mass kg
    const float J = 0.44;                  // measured from sys id kg-m^2
    const float R_WHEEL = 0.048 / ROOT_2;  // wheel radius m
    const float J_WHEEL = 0.0009;          // wheel moment of inertia kg-m^2
    const float C_MOTOR = 0.00003;         // motor damping kg-s/m^2
    const float UK_MOTOR = 0.004;          // motor dry friction N-m
    const float COF_WHEEL = 0.9;           // unitless COF

    const float KB = 0.02361;       // V-s/rad backemf
    const float KT = 0.02299;       // N-m/A torque constant
    const float RA = 0.5592;        // ohm, armature resistance
    const float GEAR_RATIO = 13.7;  // gear ratio
    const float VOLT_MAX = 24;      // V, maximum                                                                                         v
    const float P_MAX = 50;         // W, maximum power

    const float M_EFFECTIVE = M + 4 * J_WHEEL * std::pow(GEAR_RATIO / R_WHEEL, 2.0f);
    const float J_EFFECTIVE = J + 4 * J_WHEEL * std::pow((TRACKWIDTH / 2.0f) * (GEAR_RATIO / R_WHEEL), 2.0f);
    const float F_MAX = M * 9.81f * COF_WHEEL;    // maximum force allowed across all 4 wheels
    const float F_MIN_T = 10 / (2 * TRACKWIDTH);  // minimum force per wheel in the torque direction that the traction limiter is allowed to throttle to

    // Feedforward gains from fundamental system constants
    const float K_V = KB;              // Velocity feedforward gain (back EMF constant)
    const float K_VIS = C_MOTOR / KT;  // Viscous damping feedforward gain
    const float K_S = UK_MOTOR / KT;   // Static friction feedforward g

    // Tunable Parameters
    const float KP_V_XY = 1500;  // proportional gain for velocity
    const float KP_V_ROT = 30;  // proportional gain for rotational velocity
    const Pose2d KP_V{KP_V_XY, KP_V_XY, KP_V_ROT};

    const float KI_V = 0;    // integral gain for velocity

    const float IV_MAX = 120;  // maximum integral term for velocity control
    const Vector2d MIN_FORCE{-IV_MAX, -IV_MAX}, MAX_FORCE{IV_MAX, IV_MAX};


    const float KP = 0;              // proportional gain for position control
    const float VEL_TARGET = 0;      // target velocity
    const float BEYBLADE_DELAY = 0;  // delay for beyblade mode

    const float BEYBLADE_AMPLITUDE = 0;  // beyblade amplitude
    const float BEYBLADE_FREQUENCY = 0;  // beyblade frequency
    const bool BEYBLADE_FIXED_SPEED = false;

    const float LATENCY = 0.008;  // latency s
    const float DT = 0.002;       // DT in s

    // queues
    const int Q_SIZE = (LATENCY / DT);

    // matrices (big but these only get run once so yay)
    const float ikr1[3] = {GEAR_RATIO / (R_WHEEL * ROOT_2), GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};
    const float ikr2[3] = {-GEAR_RATIO / (R_WHEEL * ROOT_2), GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};
    const float ikr3[3] = {-GEAR_RATIO / (R_WHEEL * ROOT_2), -GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};
    const float ikr4[3] = {GEAR_RATIO / (R_WHEEL * ROOT_2), -GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};

    const float fkr1[4] = {R_WHEEL * ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), R_WHEEL *ROOT_2 / (4 * GEAR_RATIO)};
    const float fkr2[4] = {R_WHEEL * ROOT_2 / (4 * GEAR_RATIO), R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO)};
    const float fkr3[4] = {-R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO), -R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO), -R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO), -R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO)};

    const float fir1[3] = {ROOT_2 / 4, ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};
    const float fir2[3] = {-ROOT_2 / 4, ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};
    const float fir3[3] = {-ROOT_2 / 4, -ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};
    const float fir4[3] = {ROOT_2 / 4, -ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};

    const float ffr1[4] = {1 / ROOT_2, -1 / ROOT_2, -1 / ROOT_2, 1 / ROOT_2};
    const float ffr2[4] = {1 / ROOT_2, 1 / ROOT_2, -1 / ROOT_2, -1 / ROOT_2};
    const float ffr3[4] = {-TRACKWIDTH / 2, -TRACKWIDTH / 2, -TRACKWIDTH / 2, -TRACKWIDTH / 2};

    // Beyblade settings
    float dotThetaBeyblade = 0;  // Target beyblade velocity when chassis is not translating
    float dotThetaGain = 0;      // Amount to subtract from dot_theta_beyblade per meter/s of chassis speed

    float signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }

    float clamp(float num, float min, float max) { return std::min(std::max(num, min), max); }

    // Helper for velocity control (sawtooth function)
    float sawtooth(float freq, float amplitude) {
        // A simple sawtooth wave function for variable speed beyblade
        return amplitude;  // * (1 - fmod(freq * (std::fmod(std::clock(), 1.0f) * 2 * 3.14159265f), 1.0f));
    }

        // states to store, current position, curent velocity, and also the lastForce
    Pose2d estPosWorld{}, estVelWorld{}, lastForceLocal{};

    Vector2d lastVelWorld{}, accumForceLocal{};

public:
    float *targetVelocityHistory;  // For storing target velocity magnitudes
    Pose2d *forceHistory;          // history of past chassis forces
    ChassisController();

    ~ChassisController();
    //~YawController();
    void calculate(Pose2d targetVelLocal, float angle, float motorVelocity[4], float motorCurrent[4]);
    float calculateBeybladeVelocity(float bb_freq, float bb_amp);

    // intermediate functions
    void estimateState(Pose2d inputForces, Pose2d *eLocalVel, Pose2d *eInertialVel, Pose2d *eInertialPos);

    void calculateFeedForward(float estimatedMotorVelocity[4], float V_m_FF[4], float I_m_FF[4]);

    void velocityControl(Pose2d inputLocalVel, Vector2d estInertialVel, Pose2d estLocalVel, Vector2d lastInertialForce, Pose2d *reqLocalForce);

    void calculateTractionLimiting(Pose2d localForce, Pose2d *limitedForce);

    void calculatePowerLimiting(float V_m_FF[4], float I_m_FF[4], float T_req_m[4], float T_req_m2[4]);

    const float *inverseKinematics[4] = {ikr1, ikr2, ikr3, ikr4};
    const float *forwardKinematics[3] = {fkr1, fkr2, fkr3};
    const float *forceInverseKinematics[4] = {fir1, fir2, fir3, fir4};
    const float *forceKinematics[3] = {ffr1, ffr2, ffr3};

    float *multiplyMatrices(int rows1, int cols1, const float **mat1, float *mat2, float *result) {
        // Iterate over rows of mat1 and columns of mat2
        for (int i = 0; i < rows1; i++) {
            result[i] = 0.0f;  // Initialize to zero before accumulating
            for (int j = 0; j < cols1; j++) {
                result[i] += mat1[i][j] * mat2[j];
            }
        }
        return result;
    }


};

}  // namespace subsystems
