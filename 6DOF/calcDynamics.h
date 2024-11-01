/*calcDynamics.h*/

#include <iostream>

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
std::vector<double> getAeroForces();


// getAeroMoments
//
// Takes in .... 
//
std::vector<double> getAeroMoment();


// getAeroMoments
//
// Takes in q and computes R inverse which is the inverse of the transfomraiton from world to rocket frame
//
// double getRinv();