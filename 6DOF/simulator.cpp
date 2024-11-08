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
#include "constants.h"

q getqdot(q curr_q){}

q integrate(q curr_q){
    q k1 = getqdot(curr_q) * DT;
    q k2 = getqdot(curr_q + k1/2.0) * DT;
    q k3 = getqdot(curr_q + k2/2.0) * DT;
    q k4 = getqdot(curr_q + k3) * DT;
    q new_q = curr_q + (k1 + k2*2.0 + k3*2.0 + k4) * (1/6.0);
    return new_q;
}


bool atApogee(q curr_q){
    return (getR()*curr_q.getV())(2) <= 0;
}

double getApogee(q curr_q, double b){}

double getApogee(q curr_q){
    getApogee(curr_q, 1000);// shifting 1000 seconds to the right is plenty for a 20s flight time to make sure we never hit
}