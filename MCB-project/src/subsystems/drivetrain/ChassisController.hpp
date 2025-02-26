#pragma once
#include <cmath>
#include <deque>  // For storing the last n values
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

namespace subsystems
{
struct Pose2d
{
    float x = 0;
    float y = 0;
    float rotation = 0;

    float vectorAngle() { return atan2(y, x); }
    Pose2d rotate(float amt)
    {
        amt += vectorAngle();
        return Pose2d{magnitude() * cos(amt), magnitude() * sin(amt), rotation};
    }

    float magnitude() { return hypot(x, y); }

    Pose2d addXY(Pose2d other) { return Pose2d{x + other.x, y + other.y, rotation}; }

    Pose2d subXY(Pose2d other) { return Pose2d{x - other.x, y - other.y, rotation}; }

    Pose2d scalarMultiply(float mag) { return Pose2d{x * mag, y * mag, rotation}; }

    Pose2d elementMultiply(float mag) { return Pose2d{x * mag, y * mag, rotation * mag}; }

    Pose2d copy() { return Pose2d{x, y, rotation}; }

    std::vector<float> toVector() { return {x, y, rotation}; }
};

class ChassisController
{
private:
    // START getters and setters
    const float TRACKWIDTH = 0.5017;       // in m. We need to measure
    const float ROOT_2 = sqrt(2);          // sqrt 2
    const float M = 14.0;                  // robot mass kg
    const float J = 3.0;                   // measured from sys id kg-m^2
    const float R_WHEEL = 0.048 / ROOT_2;  // wheel radius m
    const float J_WHEEL = 0.0009;          // wheel moment of inertia kg-m^2
    const float C_MOTOR = 0.00003;         // motor damping kg-s/m^2
    const float UK_MOTOR = 0.004;          // motor dry friction N-m
    const float COF_WHEEL = 0.9;           // unitless COF

    const float M_EFFECTIVE = M + 4 * J_WHEEL / (R_WHEEL * R_WHEEL);
    const float J_EFFECTIVE = J + 4 * J_WHEEL * TRACKWIDTH / (2 * R_WHEEL);
    const float F_MAX = M * 9.81f * COF_WHEEL;      // maximum force allowed across all 4 wheels
    const float F_MIN_T = 10 / (2 * TRACKWIDTH);    // minimum force per wheel in the torque direction that the traction limiter is allowed to throttle to

    const float KB = 0.02361;       // V-s/rad backemf
    const float KT = 0.02299;       // N-m/A torque constant
    const float RA = 0.5592;        // ohm, armature resistance
    const float GEAR_RATIO = 13.7;  // gear ratio
    const float VOLT_MAX = 24;      // V, maximum                                                                                         v
    const float P_MAX = 50;         // W, maximum power

    // Feedforward gains from fundamental system constants
    const float K_V = KB;              // Velocity feedforward gain (back EMF constant)
    const float K_VIS = C_MOTOR / KT;  // Viscous damping feedforward gain
    const float K_S = UK_MOTOR / KT;   // Static friction feedforward g

    // Tunable Parameters
    const float KP_V = 1500;  // proportional gain for velocity
    const float KI_V = 50;    // integral gain for velocity

    const float IV_MAX = 120;  // maximum integral term for velocity control

    const float KP_V_ROT = 30;  // proportional gain for rotational velocity

    const float KP = 0;              // proportional gain for position control
    const float VEL_TARGET = 0;      // target velocity
    const float BEYBLADE_DELAY = 0;  // delay for beyblade mode

    const float BEYBLADE_AMPLITUDE = 0;  // beyblade amplitude
    const float BEYBLADE_FREQUENCY = 0;  // beyblade frequency
    const bool BEYBLADE_FIXED_SPEED = false;

    const float LATENCY = 0.008;  // latency s
    const float DT = 0.002;       // DT in s

    std::deque<float> targetVelocityQueue;  // For storing target velocity magnitudes

    // matrices (big but these only get run once so yay)
    const float ikr1[3] = {GEAR_RATIO / (R_WHEEL * ROOT_2), GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};
    const float ikr2[3] = {-GEAR_RATIO / (R_WHEEL * ROOT_2), GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};
    const float ikr3[3] = {-GEAR_RATIO / (R_WHEEL * ROOT_2), -GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};
    const float ikr4[3] = {GEAR_RATIO / (R_WHEEL * ROOT_2), -GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};

    const float *inverseKinematics[4] = {ikr1, ikr2, ikr3, ikr4};

    const float fkr1[4] = {R_WHEEL * ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), R_WHEEL *ROOT_2 / (4 * GEAR_RATIO)};
    const float fkr2[4] = {-R_WHEEL * ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), R_WHEEL *ROOT_2 / (4 * GEAR_RATIO)};
    const float fkr3[4] = {-R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO), -R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO), -R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO), -R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO)};

    const float *forwardKinematics[3] = {fkr1, fkr2, fkr3};

    const float fir1[3] = {ROOT_2 / 4, ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};
    const float fir2[3] = {-ROOT_2 / 4, ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};
    const float fir3[3] = {-ROOT_2 / 4, -ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};
    const float fir4[3] = {ROOT_2 / 4, -ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};

    const float *forceInverseKinematics[4] = {fir1, fir2, fir3, fir4};

    const float ffr1[4] = {1 / ROOT_2, -1 / ROOT_2, -1 / ROOT_2, 1 / ROOT_2};
    const float ffr2[4] = {-1 / ROOT_2, -1 / ROOT_2, 1 / ROOT_2, 1 / ROOT_2};
    const float ffr3[4] = {-TRACKWIDTH / 2, -TRACKWIDTH / 2, -TRACKWIDTH / 2, -TRACKWIDTH / 2};

    const float *forceKinematics[3] = {ffr1, ffr2, ffr3};

    std::deque<Pose2d *> history;  // history of past chassis forces
    // Beyblade settings
    float dotThetaBeyblade = 0;  // Target beyblade velocity when chassis is not translating
    float dotThetaGain = 0;      // Amount to subtract from dot_theta_beyblade per meter/s of chassis speed

    void multiplyMatrices(int rows1, int cols1, const float **mat1, std::vector<float> mat2, std::vector<float> &result)
    {
        // Iterate over rows of mat1 and columns of mat2
        for (int i = 0; i < rows1; i++)
        {
            result[i] = 0;  // Initialize to zero before accumulating
            for (int j = 0; j < cols1; j++)
            {
                result[i] += mat1[i][j] * mat2[j];
            }
        }
    }

    void estimateState(Pose2d &inputForces, Pose2d &eLocalVel, Pose2d &eInertialVel, Pose2d &eInertialPos);

    void calculateFeedForward(std::vector<float> &estimatedMotorVelocity, std::vector<float> &V_m_FF, std::vector<float> &I_m_FF);

    void velocityControl(Pose2d &inputLocalVel, Pose2d &estInertialVel, Pose2d &estLocalVel, Pose2d &lastInertialForce, Pose2d &reqLocalForce);

    void calculateTractionLimiting(Pose2d &localForce);

    void calculatePowerLimiting(Pose2d &localForce, std::vector<float> &V_m_FF, std::vector<float> &I_m_FF, std::vector<float> &T_req_m);

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
    void calculate(Pose2d &targetVelLocal, float &angle, float motorVelocity[4], float motorCurrent[4]);
    float calculateBeybladeVelocity(float bb_freq, float bb_amp);
};

}  // namespace subsystems
