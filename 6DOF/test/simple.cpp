#include "gtest/gtest.h"
#include "calcDynamics.h"
#include "q.h"

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

TEST(calcDynamics, simpleVSquared){
    q testq = q(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), )
}