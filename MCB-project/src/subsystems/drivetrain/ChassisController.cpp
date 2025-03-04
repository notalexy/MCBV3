#include "ChassisController.hpp"

#include <cmath>
#include <iostream>
#include <string>

// TODO: Calculate function that takes in target x, y velocities as well as a target angle as inputs, outputs 4 motor current summations

float test1;
float test2;
float test3;
float test4;
float test5;
float test6;
float test7;
float test8;
float test9;

namespace subsystems {
ChassisController::ChassisController() {
    targetVelocityHistory = new float[Q_SIZE];
    forceHistory = new Pose2d[Q_SIZE];
    for(int i = 0; i < Q_SIZE; i++){
        forceHistory[i] = Pose2d();
    }
}

// Function to calculate chassis state using historical force/torque data and latency
void ChassisController::estimateState(Pose2d inputForces, Pose2d* eLocalVel, Pose2d* eInertialVel, Pose2d* eInertialPos) {
    // // Store the current values in the history
    for(int i = Q_SIZE; i > 0; i--) {
        forceHistory[i] = forceHistory[i-1];
    }

    //newest value at 0
    forceHistory[0] = inputForces;

    // For each historical value of Fx, Fy, Tz, calculate the estimates
    for (int i = Q_SIZE-1; i >= 0; i--) {
        // Estimated angular velocity (theta_dot)
        *eInertialVel = eInertialVel->addRotation(forceHistory[i].getRotation() * DT / J_EFFECTIVE);

        // Update theta estimate (added DT because i think alex Y did it wrong)
        *eInertialPos = eInertialPos->addRotation(eInertialVel->getRotation() * DT);

        // Update velocity
        *eInertialVel = eInertialVel->addXY(forceHistory[i].rotate(-eInertialPos->getRotation()).scalarMultiply(DT / M_EFFECTIVE));

        // update positions
        *eInertialPos = eInertialPos->addXY(eInertialVel->scalarMultiply(DT));
    }

    *eLocalVel = eInertialVel->rotate(eInertialPos->getRotation());
}

float ChassisController::calculateBeybladeVelocity(float bb_freq, float bb_amp) {
    // Get the target velocity commands (or position commands) from the controller
    // This part would integrate the user input or other controller logic to set target velocity

    float velmagMax = 0;
    for (int i = 0; i < Q_SIZE; i++) {
        velmagMax = std::max(velmagMax, targetVelocityHistory[i]);  // Find max velocity magnitude in the last bb_delay seconds
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
    // targetVelocityQueue.emplace_back(VEL_TARGET);
    // if (targetVelocityQueue.size() > BEYBLADE_DELAY / DT) {
    //     targetVelocityQueue.pop_front();
    // }

    return dotThetaBeyblade;  // Return the required angular velocity (dot_theta_req)
}

Pose2d lastInertialVel{}, accumLocalForce{};
// Function to estimate input errors in inertial frame
void ChassisController::velocityControl(Pose2d inputLocalVel, Pose2d estInertialVel, Pose2d estLocalVel, Pose2d lastInertialForce, Pose2d* reqLocalForce) {
    Pose2d estInertialForce = estInertialVel.subXY(lastInertialVel).scalarMultiply(M_EFFECTIVE / DT);

    // Compute the error between the last and the estimated forces in the inertial frame
    Pose2d errorInertialForce = lastInertialForce.subXY(estInertialForce);

    // Update integral error terms
    accumLocalForce = accumLocalForce.addXY(errorInertialForce.subXY(accumLocalForce).scalarMultiply(KI_V * DT));

    // update the required local force
    *reqLocalForce = inputLocalVel.subXY(estLocalVel).scalarMultiply(KP_V).addXY(accumLocalForce);

    // Store current forces for the next iteration
    lastInertialVel = estInertialVel;
}

// Get the feed forward variables for each motor based on the previously calculaed inverse kinematic estimated theta (see feedforwards)
void ChassisController::calculateFeedForward(float estimatedMotorVelocity[4], float V_m_FF[4], float I_m_FF[4]) {
    float vel;
    for (int i = 0; i < 4; ++i) {
        vel = estimatedMotorVelocity[i];
        V_m_FF[i] = K_V * vel;
        I_m_FF[i] = K_VIS * vel + K_S * signum(vel);
    }
}

void ChassisController::calculateTractionLimiting(Pose2d localForce, Pose2d* limitedForce) {
    float beybladeCommand = 0;  // calculateBeybladeVelocity(0.0f, 0.0f);  // not sure what I should pass into here

    float motorTorque[4];

    float forceVector[] = {localForce.getX(), localForce.getY(), localForce.getRotation()};

    multiplyMatrices(4, 3, forceInverseKinematics, forceVector, motorTorque);

    float largest = motorTorque[0];

    // Manipulate F_largest based on whether the beyblade torque command is positive or negative
    for (int i = 1; i < 4; i++) {  // if beybladeCommand > 0, find smallest, vice versa
        if (beybladeCommand > 0 ^ motorTorque[i] > largest) {
            largest = motorTorque[i];
        }
    }

    float F_too_much = std::max(std::abs(largest) - F_MAX / 4, 0.0f);  // clamp between 0 and infinity

    float T_req = localForce.getRotation();

    float T_req_throttled = signum(T_req) * std::min(std::abs(T_req), 2 * TRACKWIDTH * std::max(T_req / (2 * TRACKWIDTH) - F_too_much, F_MIN_T));

    float F_lat[4];

    // 2 rows to not do torque
    multiplyMatrices(4, 2, forceInverseKinematics, forceVector, F_lat);

    largest = std::abs(F_lat[0]);

    // Manipulate F_largest based on whether the beyblade torque command is positive or negative
    for (int i = 1; i < 4; i++) {  // if beybladeCommand > 0, find smallest, vice versa
        if (std::abs(F_lat[i]) > largest) {
            largest = std::abs(F_lat[i]);
        }
    }
    // since 1/0 is inf, 1/0.00000000000001 is inf basically, do nothing
    if (largest == 0) return;

    *limitedForce = localForce.scalarMultiply(std::clamp((F_MAX / 4 - T_req_throttled / (2 * TRACKWIDTH)) * 1 / largest, 0.0f, 1.0f));
}

// Calculates scaling factor based on the equations in the notion
void ChassisController::calculatePowerLimiting(float V_m_FF[4], float I_m_FF[4], float T_req_m[4], float T_req_m2[4]){



    // // Get all the summations out of the way first
    float aSum = 0, bSumFirst = 0, bSumSecond = 0, cSumFirst = 0, cSumSecond = 0;
    for (int i = 0; i < 4; i++) {
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
    float s_scaling = 1.0f;//std::clamp((-b + sqrt(b * b - 4 * a * c)) / (2 * a), 0.0f, 1.0f);
    
    // And finally get T_req_m_throttled based on the scaling factor
    for (int i = 0; i < 4; ++i) {
        T_req_m2[i] = s_scaling * T_req_m[i];
    }
}

Pose2d currentInertialPosition{}, currentLocalVelocity{}, localForce{}, lastInertialForce{};

//std::vector<float> V_m_FF{4, 0.0f}, I_m_FF{4, 0.0f}, estimatedMotorVelocity{4, 0.0f}, motorTorque{4, 0.0f};  // motor feedforwards

float V_m_FF[4], I_m_FF[4], estimatedMotorVelocity[4], motorTorque[4];  // motor feedforwards

void ChassisController::calculate(Pose2d targetVelLocal, float angle, float motorVelocity[4], float motorCurrent[4]) {


    float vec[3];
    multiplyMatrices(3, 4, forwardKinematics, motorVelocity, vec);

    Pose2d estimatedLocalVelocity{vec[0], vec[1], vec[2]};

    test2 = estimatedLocalVelocity.getY();

    Pose2d estimatedInertialPosition{0.0f, 0.0f, angle};

    Pose2d estimatedInertialVelocity = estimatedLocalVelocity.rotate(-angle);
    
    estimateState(localForce, &estimatedLocalVelocity, &estimatedInertialVelocity, &estimatedInertialPosition);

   // test2 = motorVelocity[0];

    float localVelArr[] = {estimatedLocalVelocity.getX(), estimatedLocalVelocity.getY(), estimatedLocalVelocity.getRotation()};

    test3 = estimatedInertialVelocity.getY();

    // inverse kinematics
    multiplyMatrices(4, 3, inverseKinematics, localVelArr, estimatedMotorVelocity);

    test4 = estimatedMotorVelocity[0];

    // calculateBeybladeVelocity(0.0f, 0.0f);  // should be changed

    calculateFeedForward(estimatedMotorVelocity, V_m_FF, I_m_FF);

    test5 = estimatedLocalVelocity.getY();
    // get last inertial force too
    Pose2d lastInertialForce = localForce.rotate(-estimatedInertialPosition.getRotation());

    // // First, estimate the input errors
    velocityControl(targetVelLocal, estimatedInertialVelocity, estimatedLocalVelocity, lastInertialForce, &localForce);

    test6 = I_m_FF[0];
    
    test1 = localForce.getY();
   // Pose2d tractionLimitedForce;

    //calculateTractionLimiting(localForce, &localForce);

    //test6 = tractionLimitedForce.getX();
    // std::vector<float> vec3(3, 0.0f);

    float localForceMotorArr[] = {localForce.getX()*R_WHEEL/GEAR_RATIO, localForce.getY()*R_WHEEL/GEAR_RATIO, localForce.getRotation()*R_WHEEL/GEAR_RATIO};

    multiplyMatrices(4, 3, forceInverseKinematics, localForceMotorArr, motorTorque);

    //calculatePowerLimiting(V_m_FF, I_m_FF, motorTorque, motorTorque);

    test7 = motorTorque[0];

    float localForceChassisArr[3];

    multiplyMatrices(3, 4, forceKinematics, motorTorque, localForceChassisArr);

    test8 = localForceChassisArr[0];

    localForce = Pose2d{localForceChassisArr[0], localForceChassisArr[1], localForceChassisArr[2]};
    
    test9 = localForce.getX();
    // set motor currents
    for (int i = 0; i < 4; i++) {
        motorCurrent[i] = motorTorque[i] / KT + I_m_FF[i];
    }
    //test9 = motorCurrent[0];
}

}  // namespace subsystems
