#include "gtest/gtest.h"
#include <iostream>
#include "calcDynamics.h"
#include <Eigen/Core>
#include "q.h"
#include "aeroData.h"

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
    q testq(v, t, o, 0.0, 0.0);
    double result = getAlpha(testq);
    EXPECT_EQ(result, 0);
}

TEST(GetAlpha, bigVx){
    Eigen::Vector3d v(100000000000, 2, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, t, o, 0.0, 0.0);
    double result = getAlpha(testq);
    EXPECT_NEAR(result, 0, 0.01);
}

TEST(GetAlpha, noV){
    Eigen::Vector3d v(0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, t, o, 0.0, 0.0);
    double result = getAlpha(testq);
    EXPECT_EQ(result, 0);
}

TEST(AeroForces, noV){
    Eigen::Vector3d v(0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, t, o, 0.0, 0.0);
    Eigen::Vector3d result = getAeroForces(testq);
    Eigen::Vector3d answer(0, 0, 0);
    EXPECT_EQ(result, answer);
}

TEST(AeroForces, BigX){
    Eigen::Vector3d v(100000, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Vector3d o(0, 0, 0);
    q testq(v, t, o, 0.0, 0.0);
    Eigen::Vector3d result = getAeroForces(testq);
    Eigen::Vector3d answer(0, 0, 0);
    EXPECT_EQ(result, answer);
}
