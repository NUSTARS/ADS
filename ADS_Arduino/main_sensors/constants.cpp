/*constants.cpp*/
#include "constants.h"

//
// Constants that will be used throuhout the 6DOF simulator.
// THIS AND CONSTANTS.H ARE THE ONLY CODE THAT SHOULD BE TOUCHED DAY OF FLIGHT
// Should be measured and changed on the day of flight to represent the actual rocket
//
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025
//

// control function for ADS actuation, ramps from F=0 at t=0 to F=1 at t=f (a is defined by the function)
double F_function(double t){ 
    double res = 100*t;
    if(res > 1.0){
        res = 1.0;
    }
    else if(res < 0.0){
        res = 0.0;
    }
    return res;
}