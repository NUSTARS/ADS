/*simulator.h*/
//
// preforms all of the dynamicds simulation for integration
//
//

// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <iostream>
#include <Eigen/Core>
#include "q.h"
#include "wind.h"
#include <vector>

// getqdot
//
// given a q, returns the time derivative based on the EOM
//
q getqdot(q curr_q, Wind& wind);

// integrate
//
// given a q, returns q at the next timestep using qdot and dt (RK4)
//
q integrate(q curr_q, Wind& wind);


// atApogee
//
// takes in the current state and returns a boolean based on whether apogee
// has been reached (defined as starting to move downwards here)
//
bool atApogee(q curr_q);

// getApogee
//
// returns the apogee reached given an IC and a b value. Overloaded to take a b or no b
//
double getApogee(q curr_q, double b);
double getApogee(q curr_q);

q  getqdot_testing(q curr_q);
double getApogee_testing(q curr_q);

#endif



