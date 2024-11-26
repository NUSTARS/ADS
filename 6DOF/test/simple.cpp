#include "gtest/gtest.h"
#include <iostream>
#include "calcDynamics.h"
#include <Eigen/Core>
#include "q.h"
#include "aeroData.h"
#include "cmath"

TEST(One, EqualsOne)
{
    EXPECT_EQ(1, 1);
}

TEST(MathOperations, AdditionWorks) {
    int result = 2 + 2;
    EXPECT_EQ(result, 4); 
}

TEST(MathOperations, MultiplicationWorks) {
    int result = 2 * 3;
    EXPECT_EQ(result, 6); // Passes if `result == 6`
}

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

    


    Eigen::Vector3d v1(0, 0, 0);
    Eigen::Vector3d t1(0, 0.2125811, 0);
    Eigen::Vector3d o1(0, 0, 0);
    q qrot(v1, o1, t1, 845, 0.0);


    Eigen::Vector3d vel(650, 139, 0);
    Eigen::Matrix3d rinv = getR(qrot);
    std::cout << rinv << std::endl;
    std::cout << vel.transpose() * rinv << std::endl;

    
    Eigen::Vector3d v(650, 139, 0);
    Eigen::Vector3d t(0, 0.0003490659, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(vel.transpose() * rinv, o, t, 845, 0.0);


    Eigen::Vector3d result = getAeroForces(testq);
    std::cout << result << std::endl;
    ASSERT_NEAR(getRho(845), 0.00232, 0.000232);
    ASSERT_NEAR(result(0), -42.98, 4.298);
    
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

