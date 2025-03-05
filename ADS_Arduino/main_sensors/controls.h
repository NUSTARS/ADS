#ifndef CONTROLS_H
#define CONTROLS_H


// #include <iostream>
//#include <Eigen/Core>
#include <ArduinoEigen.h>
#include "q.h"


//
// createRange
//
//
// Takes in start, end and timestep to create discrete vector
//
std::vector<double> createRange(double start, double end, double step);

//
// binary_search
//
// Takes in range of b values and determines b that fits apogee based on error and current state
//
double binary_search(const std::vector<double>& b, 
                            double apogee, 
                            double err,
                            q states);
//
// generateRampingFunction
//
// creates ramping function of a given duration with delay b and ramping constant f
//                              
std::vector<double> generateRampingFunction(double duration, double b, double f, double timestep);

//
// findU
//
// generates control signal based on current state, desired apogee and acceptable error
//  
double findU(q states, double apogee, double err);

#endif 