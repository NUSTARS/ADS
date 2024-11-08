/*simulator.cpp*/
//
// preforms all of the dynamicds simulation for integration
//
//

// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025

#include <iostream>
#include <Eigen/Core>
#include "q.h"
#include "calcDynamics.h"

q getqdot(q curr_q){}

q integrate(q curr_q){}


bool atApogee(q curr_q){
    return (getRinv()*curr_q.getV())(2) <= 0;
}

double getApogee(q curr_q, double b){}
double getApogee(q curr_q){}