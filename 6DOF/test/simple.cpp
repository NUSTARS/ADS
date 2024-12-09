#include "gtest/gtest.h"
#include <iostream>
#include "calcDynamics.h"
#include <Eigen/Core>
#include "q.h"
#include "aeroData.h"
#include "cmath"
#include "constants.h"

TEST(VSquared, ZeroV){
    Eigen::Vector3d v(0, 0, 0);
    q testq(v, v, v, 0.0, 0.0);
    // std::cout << getV_Squared(testq) << std::endl;
    double result = getV_Mag(testq);
    EXPECT_EQ(result, 0);
}

TEST(VSquared, SmallV){
    Eigen::Vector3d v(2, 3, 4);
    q testq(v, v, v, 0.0, 0.0);
    double result = getV_Mag(testq);
    EXPECT_EQ(result, std::sqrt(29));
}

TEST(GetAlpha, onlyX){
    Eigen::Vector3d v(2, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, o, t, 0.0, 0.0);
    double result = getAlpha(testq);
    EXPECT_EQ(result, 0);
}

TEST(GetAlpha, bigVx){
    Eigen::Vector3d v(100000000000, 2, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, o, t, 0.0, 0.0);
    double result = getAlpha(testq);
    EXPECT_NEAR(result, 0, 0.01);
}

TEST(GetAlpha, noV){
    Eigen::Vector3d v(0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, o, t, 0.0, 0.0);
    double result = getAlpha(testq);
    EXPECT_EQ(result, 0);
}

TEST(AeroForces, noV){
    Eigen::Vector3d v(0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, o, t, 0.0, 0.0);
    Eigen::Vector3d result = getAeroForces(testq);
    Eigen::Vector3d answer(0, 0, 0);
    EXPECT_EQ(result, answer);
}

TEST(AeroForces, BigX){
    Eigen::Vector3d v(1000000, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, o, t, 0.0, 0.0);
    Eigen::Vector3d result = getAeroForces(testq);
    EXPECT_TRUE(-result(0) > result(1));
    EXPECT_TRUE(-result(0) > result(2));
    EXPECT_EQ(result(1), 0);
    EXPECT_EQ(result(2), 0);
}

TEST(AeroForces, BurnoutX){

    double OR_LATERAL_VELOCITY = 73.969; //[ft/s]
    double OR_VERTICAL_VELOCITY = 721.373; //[ft/s]
    double OR_PITCH_RATE = -1.73E-04; // [r/s]
    double OR_YAW_RATE = -5.27E-06; // [r/s]
    double OR_AZIMUTH = 0.017; // [deg]
    double OR_ZENITH = 84.166; // [deg]
    double initial_h = 964.927; // [ft]
  
    Eigen::Vector3d initial_v_world(0, -OR_LATERAL_VELOCITY, OR_VERTICAL_VELOCITY); // good
    Eigen::Vector3d initial_omega(0, OR_PITCH_RATE*2*M_PI, OR_YAW_RATE*2*M_PI); // good
    Eigen::Vector3d initial_theta(OR_AZIMUTH*M_PI/180.0, 0, (90-OR_ZENITH)*M_PI/180.0); // not sure

    Eigen::Vector3d initial_v_body = getRinv(q(Eigen::Vector3d(0,0,0), initial_omega, initial_theta, initial_h, 0))*initial_v_world; 

    // Eigen::Matrix3d rinv = getRinv(q(Eigen::Vector3d(0,0,0), initial_omega, initial_theta, initial_h, 0)); 
    std::cout << initial_v_body << std::endl;

    q testq(initial_v_body, initial_omega, initial_theta, initial_h, 0);

    Eigen::Vector3d result = getAeroForces(testq);

    double Fd = -0.5*20.831/144.0*0.00232*725.155*725.155*0.594;

    double Cn_calc = getCN(getV_Mag(testq)*getV_Mag(testq),getAlpha(testq),0,initial_h);

    std::cout << Fd << std::endl;
    EXPECT_EQ(A, 20.831/144.0);
    EXPECT_NEAR(getAlpha(testq)*(180.0/M_PI), 0.019, 0.001);
    EXPECT_NEAR(getV_Mag(testq), 725.155, 0.1); //total mag the same
    // EXPECT_NEAR(Cn_calc, 0.004, 0.0001);
    EXPECT_NEAR(result(0), Fd, 1.0);
    
}

TEST(R, DoNothing){
    Eigen::Vector3d v(0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, o, t, 0.0, 0.0);
    Eigen::Matrix3d result = getR(testq);
    Eigen::Matrix3d answer{{0, 0, 1},  
                           {0, -1, 0}, 
                           {1, 0, 0}};
    EXPECT_EQ(result, answer);
}

TEST(R, pitch90){
    Eigen::Vector3d v(0, 0, 0);
    Eigen::Vector3d t(0, 3.14159/2, 0.0);
    Eigen::Vector3d o(0, 0, 0);
    q testqy(v, o, t, 0.0, 0.0);
    // std::cout << testqy.getOmega() << std::endl;
    Eigen::Matrix3d result = getR(testqy);
    // Eigen::Matrix3d answer{{1, 0, 0}, 
                        //    {0, 1, 0}, 
                        //    {0, 0, 1}};

    EXPECT_NEAR(result(1,1), -1, 0.1);
}

TEST(R, roll180){
    Eigen::Vector3d v(0, 0, 0);
    Eigen::Vector3d t(0, 0, 3.14159);
    Eigen::Vector3d o(0, 0, 0);
    q testqy(v, o, t, 0.0, 0.0);
    // std::cout << testqy.getOmega() << std::endl;
    Eigen::Matrix3d result = getR(testqy);
    // Eigen::Matrix3d answer{{1, 0, 0}, 
                        //    {0, 1, 0}, 
                        //    {0, 0, 1}};

    EXPECT_NEAR(result(1,1), 1, 0.1);
}

