/*calcDynamics.h*/

#include <iostream>
#include <Eigen/Core>


// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
//  NUSTARS NSL 2025



// getVMag
//
// Takes in q
double getV_Squared(q);

// getAlpha
//
// Takes in q 
double getAlpha(q);


// getAeroForces
//
// Takes in .... 
//
Eigen::Vector3d getAeroForces(q);


// getAeroMoments
//
// Takes in .... 
//
Eigen::Vector3d getAeroMoments(q);


// getAeroMoments
//
// Takes in q and computes R inverse which is the inverse of the transfomraiton from world to rocket frame
//
Eigen::Matrix3d getR(q);

Eigen::Vector3d calcWindNoise(q, Eigen::Vector2d old_w);

double a(int k, double past_A);