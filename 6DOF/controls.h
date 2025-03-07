#ifndef CONTROLS_H
#define CONTROLS_H


#include <iostream>
#include <Eigen/Core>
#include "q.h"
#include <vector>

//
// findU
//
// generates control signal based on current state, desired apogee and acceptable error
//  
double findU(q states, double apogee, double err);

#endif 