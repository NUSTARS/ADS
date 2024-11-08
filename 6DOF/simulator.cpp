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
    q k1 =  0.1 * getqdot(curr_q);
    q k2 = curr_q + k1;
    //q k2 = DT * getqdot(curr_q + k1/2.);
    q k3 = DT * getqdot(curr_q + k2/2.);
    q k4 = DT * getqdot(curr_q + k3);
    q new_q = curr_q + (1/6.) * (k1 + 2.0*k2 + 2.0*k3 + k4);
    return new_q;
}


bool atApogee(q curr_q){
    return (getR()*curr_q.getV())(2) <= 0;
}

double getApogee(q curr_q, double b){}

double getApogee(q curr_q){
    getApogee(curr_q, 1000);// shifting 1000 seconds to the right is plenty for a 20s flight time to make sure we never hit
}