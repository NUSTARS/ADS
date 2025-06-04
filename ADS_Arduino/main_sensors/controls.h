#ifndef CONTROLS_H
#define CONTROLS_H


// #include <iostream>
//#include <Eigen/Core>
#include <ArduinoEigen.h>
#include "q.h"

//
// findU
//
// generates control signal based on current state, desired apogee and acceptable error
//  
double findU(q states, double apogee, double err);

#endif 