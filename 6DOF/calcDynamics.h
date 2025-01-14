/*calcDynamics.h*/
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
//  NUSTARS NSL 2025

#ifndef CALCDYNAMICS_H
#define CALCDYNAMICS_H

#include <iostream>
#include <Eigen/Core>
#include "q.h"


// getVMag
//
// Takes in q
double getV_Mag(q);

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


// getR
//
// Takes in q and computes R which is the transformation from world to rocket frame
//
Eigen::Matrix3d getR(q);

// getRinv
//
// Takes in q and computes inverse of R which is the transformation from rocket to world frame
//
Eigen::Matrix3d getRinv(q curr_q);


// calcWindNoise
//
// takes in the past two white noise Ws and generates a pink noise signal
//
Eigen::Vector2d calcWindNoise(q, Eigen::Vector2d* old_w);


// a
//
// takes in which a it is and the a before and calculates the current a
//
double a(int k, double past_A);

#endif // calcDynamics_H