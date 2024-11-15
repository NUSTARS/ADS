/*calcDynamics.h*/
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
//  NUSTARS NSL 2025

#ifndef CALCDYNAMICS_H
#define CALCDYNAMICS_H

#include <iostream>
#include <Eigen/Core>


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
// Takes in q and computes R inverse which is the inverse of the transformation from world to rocket frame
//
Eigen::Matrix3d getR(q);


// calcWindNoise
//
// takes in the past two white noise Ws and generates a pink noise signal
//
Eigen::Vector3d calcWindNoise(q, Eigen::Vector2d* old_w);


// a
//
// takes in which a it is and the a before and calculates the current a
//
double a(int k, double past_A);

#endif // calcDynamics_H