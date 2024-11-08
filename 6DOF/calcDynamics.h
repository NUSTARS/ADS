/*calcDynamics.h*/

#include <iostream>
#include <Eigen/Core>


// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025


// getAlpha
//
// Takes in q 
double getAlpha();


// getAeroForces
//
// Takes in .... 
//
Eigen::Vector3d getAeroForces();


// getAeroMoments
//
// Takes in .... 
//
Eigen::Vector3d getAeroMoment();


// getAeroMoments
//
// Takes in q and computes R inverse which is the inverse of the transfomraiton from world to rocket frame
//
Eigen::Matrix3d getR();

Eigen::Vector3d calcWindNoise();