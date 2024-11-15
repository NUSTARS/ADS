#ifndef CONTROLS_H
#define CONTROLS_H


#include <iostream>
#include <Eigen/Core>
#include "q.h"

std::vector<double> createRange(double start, double end, double step);

double binary_search(const std::vector<double>& b, 
                            double apogee, 
                            double err,
                            q states);
                            
std::vector<double> generateRampingFunction(double duration, double b, double f, double timestep);

double find_u(q states, double apogee, double err);

#endif 