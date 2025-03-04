#include <gtest/gtest.h>
#include "subsystems/drivetrain/ChassisController.hpp"

using namespace subsystems;

class ChassisControllerTest : public ::testing::Test {
protected:
    ChassisController controller;
    Pose2d localVel, inertialVel, inertialPos;

};

void observerRun(){
    
}
TEST_F(ChassisControllerTest, EstimateStateHistory) {

    controller.estimateState(Pose2d(1,1,3), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(2,2,4), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(3,3,7), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(4,4,7), &localVel, &inertialVel, &inertialPos);

    for(int i = 0; i < 4; i++){
        EXPECT_NEAR(4-i, controller.forceHistory[i].getX(), 0.001); //if x is right they all are probably are
    }

    //set these
    localVel = Pose2d(1, 2, 0);
    inertialVel = Pose2d(-0.6316, 2.1450, 3);
    inertialPos = Pose2d(0, 0, 0.75);

    controller.estimateState(Pose2d(6,7,7), &localVel, &inertialVel, &inertialPos);
}
TEST_F(ChassisControllerTest, EstimateStateHistory2) {
    
    controller.estimateState(Pose2d(1,1,3), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(2,2,4), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(3,3,7), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(4,4,7), &localVel, &inertialVel, &inertialPos);
    //set these
    localVel = Pose2d(1, 2, 0);
    inertialVel = Pose2d(-0.6316, 2.1450, 3);
    inertialPos = Pose2d(0, 0, 0.75);

    controller.estimateState(Pose2d(6,7,7), &localVel, &inertialVel, &inertialPos);

    EXPECT_NEAR(1.0508f, localVel.getX(), 0.0001f);
    EXPECT_NEAR(1.9786f, localVel.getY(), 0.0001f);
    EXPECT_NEAR(3.0045f, localVel.getRotation(), 0.0001f);
}
TEST_F(ChassisControllerTest, EstimateStateHistory3) {

    controller.estimateState(Pose2d(1,1,3), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(2,2,4), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(3,3,7), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(4,4,7), &localVel, &inertialVel, &inertialPos);
    //set these
    localVel = Pose2d(1, 2, 0);
    inertialVel = Pose2d(-0.6316, 2.1450, 3);
    inertialPos = Pose2d(0, 0, 0.75);

    controller.estimateState(Pose2d(6,7,7), &localVel, &inertialVel, &inertialPos);

    EXPECT_NEAR(-0.6317f, inertialVel.getX(), 0.0001f);
    EXPECT_NEAR(2.1494f, inertialVel.getY(), 0.0001f);
    EXPECT_NEAR(3.0045f, inertialVel.getRotation(), 0.0001f);
}
TEST_F(ChassisControllerTest, EstimateStateHistory4) {

    controller.estimateState(Pose2d(1,1,3), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(2,2,4), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(3,3,7), &localVel, &inertialVel, &inertialPos);
    controller.estimateState(Pose2d(4,4,7), &localVel, &inertialVel, &inertialPos);
    //set these
    localVel = Pose2d(1, 2, 0);
    inertialVel = Pose2d(-0.6316, 2.1450, 3);
    inertialPos = Pose2d(0, 0, 0.75);

    controller.estimateState(Pose2d(6,7,7), &localVel, &inertialVel, &inertialPos);

    EXPECT_NEAR(-0.0051f, inertialPos.getX(), 0.0001f);
    EXPECT_NEAR(0.0172f, inertialPos.getY(), 0.0001f);
    EXPECT_NEAR(0.7740f, inertialPos.getRotation(), 0.0001f);
}




// TEST_F(ChassisControllerTest, CalculateBeybladeVelocity) {
//     float vel = controller.calculateBeybladeVelocity(1.0, 2.0);
//     EXPECT_NEAR(vel, 0.0, 10.0); // Arbitrary range check
// }

TEST_F(ChassisControllerTest, VelocityControl) {
    Pose2d inputLocalVel(1.0, 0.5, 0.2);
    Vector2d estInertialVel(0.9, 0.4);
    Pose2d estLocalVel(0.8, 0.3, 0.1);
    Vector2d lastInertialForce(0.2, 0.1);
    Pose2d reqLocalForce;
    controller.velocityControl(inputLocalVel, estInertialVel, estLocalVel, lastInertialForce, &reqLocalForce);
    EXPECT_NEAR(reqLocalForce.getX(), 0.2, 0.1);
    EXPECT_NEAR(reqLocalForce.getY(), 0.2, 0.1);
}

TEST_F(ChassisControllerTest, CalculateFeedForward) {
    float estimatedMotorVelocity[4] = {1.0, 2.0, 3.0, 4.0};
    float V_m_FF[4], I_m_FF[4];
    controller.calculateFeedForward(estimatedMotorVelocity, V_m_FF, I_m_FF);
    for (int i = 0; i < 4; i++) {
        EXPECT_GT(V_m_FF[i], 0);
        EXPECT_GT(I_m_FF[i], 0);
    }
}

TEST_F(ChassisControllerTest, CalculateTractionLimiting) {
    Pose2d localForce(1.0, 1.0, 0.1);
    Pose2d limitedForce;
    controller.calculateTractionLimiting(localForce, &limitedForce);
    EXPECT_LE(limitedForce.getX(), localForce.getX());
    EXPECT_LE(limitedForce.getY(), localForce.getY());
}

TEST_F(ChassisControllerTest, CalculatePowerLimiting) {
    float V_m_FF[4] = {1.0, 1.2, 1.1, 1.3};
    float I_m_FF[4] = {0.5, 0.6, 0.55, 0.65};
    float T_req_m[4] = {2.0, 2.5, 2.2, 2.8};
    float T_req_m2[4];
    controller.calculatePowerLimiting(V_m_FF, I_m_FF, T_req_m, T_req_m2);
    for (int i = 0; i < 4; i++) {
        EXPECT_LE(T_req_m2[i], T_req_m[i]);
    }
}
