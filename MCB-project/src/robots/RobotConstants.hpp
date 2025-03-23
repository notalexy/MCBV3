#pragma once
#include "tap/algorithms/smooth_pid.hpp"

constexpr float ROOT_2 = 1.41421356;          // sqrt 2
constexpr float PI = 3.14159265;
int signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }

typedef struct {
    const float TRACKWIDTH;       // in m. We need to measure
    const float M;                  // robot mass kg
    const float J;                  // measured from sys id kg-m^2
    const float R_WHEEL;  // wheel radius m
    const float J_WHEEL;          // wheel moment of inertia kg-m^2
    const float C_MOTOR;         // motor damping kg-s/m^2
    const float UK_MOTOR;          // motor dry friction N-m
    const float COF_WHEEL;           // unitless COF

    const float KB;       // V-s/rad backemf
    const float KT;       // N-m/A torque constant
    const float RA;        // ohm, armature resistance
    const float GEAR_RATIO;  // gear ratio
    const float VOLT_MAX;      // V, maximum                                                                                         v

    // Tunable Parameters
    const float KP_V_XY;  // proportional gain for velocity
    const float KP_V_ROT;  // proportional gain for rotational velocity
    const float KI_V;    // integral gain for velocity
    const float IV_MAX;  // maximum integral term for velocity control

    const float KP;              // proportional gain for position control
    const float VEL_TARGET;      // target velocity
    const float BEYBLADE_DELAY;  // delay for beyblade mode

    const float BEYBLADE_AMPLITUDE;  // beyblade amplitude
    const float BEYBLADE_FREQUENCY;  // beyblade frequency
    const bool BEYBLADE_FIXED_SPEED;

    const float LATENCY;  // latency s
    const float DT;       // DT in s
} ChassisConstants;

typedef struct {
    struct Gimbal {
        const int YAW_MOTOR_MAX_SPEED;     // TODO: Make this value relevent
        const int YAW_MOTOR_MAX_VOLTAGE;  // Should be the voltage of the battery. Unless the motor maxes out below that.
        // //TODO: Check the datasheets
        
        // standard looks down 17 degrees, 15 is safe
        const float MAX_PITCH_UP;
        // looks up 20, 18 is safe
        const float MAX_PITCH_DOWN;

        const float YAW_OFFSET;
        
        const float dt;
        const float PITCH_OFFSET;  // to make gimbal horizontal when told to go to 0
        
        // for sysid
        const int YAW_DIST_RANGE;
        const int PITCH_DIST_RANGE;
    } GIMBAL;
    
    struct Pitch {
        // Physical constants
        const float KB;                          // V-rad/s
        const float RA;                          // ohm
        const float RATIO;                           // unitless
        const float VOLT_MAX;                     // V
        const float ACCEL_MAX;                    // rad/s

        // Position controller constants
        const float KP;  // sec^-1

        // Feedforward constants
        const float KSTATIC;    // A
        const float KF;        //-0.001;                     // A

        // Velocity feedback
        const float KPV;                      // 0.3                  // A-s/rad
        const float KIV;                        // 2                 // A/rad
        const float IV_MAX;                   // units TBD
        const float INT_THRESH;  // V
        const float TAKEBACK;                // unitless
    } PITCH;

    struct Yaw {
        // Physical constants
        const float C;                           // kg-s/m^2
        const float J;                           // kg-m^2
        const float UK;                           // N-m
        const float KB;                          // V-rad/s
        const float KT;                          // N-m/A
        const float RA;                          // ohm
        const float RATIO;                           // unitless
        const float VOLT_MAX;                       // V

        // Position controller constants
        const float KP;  // 10.5;  // sec^-1

        // Feedforward constants
        const float A_SCALE;                       // 0.8            // unitless

        // Gain scheduling
        const float KDT;     // unitless
        const float KDT_REV;  // unitless

        // Velocity feedback
        const float KPV;                      // A-s/rad
        const float KIV;                       // A/rad
        const float IV_MAX;               // units TBD
        const float INT_THRESH;  // V
        const float TAKEBACK;                // unitless

        const float A_DECEL;  // experimental per Alex_Y
    } YAW;
} GimbalConstants;

typedef struct {
    const int INDEXER_MOTOR_MAX_SPEED; // With the 2006, this should give
    const float REV_PER_BALL; // revolutions per ball = ratio / chambers
    const float UNJAM_BALL_PER_SECOND; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
    const tap::algorithms::SmoothPidConfig PID_CONF_INDEX;
} IndexerConstants;

typedef struct {
    const int FLYWHEEL_MOTOR_MAX_RPM;  // We had 5000 last year, and we can go 30/18 times as fast. So 5000 * 30/18
    const int FLYWHEEL_RADIUS_MM;
    
    const tap::algorithms::SmoothPidConfig PID_CONF_FLYWHEEL;
} FlywheelConstants;
