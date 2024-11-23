/*constants.h*/

//
// Constants that will be used throuhout the 6DOF simulator.
// THIS AND CONSTANTS.CPP ARE THE ONLY CODE THAT SHOULD BE TOUCHED DAY OF FLIGHT
// Should be measured and changed on the day of flight to represent the actual rocket
//
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Eigen/Core>

//
// Mechanical constants
//
const double Ix = (103.26)/(32.17*144); // Moment of inertia about the X-axis [slugs*ft^2]
const double Iy = 15906.0/(32.17*144); // Moment of inertia about the Y-axis [slugs*ft^2]
const double Iz = 15906.0/(32.17*144); // Moment of inertia about the Z-axis [slugs*ft^2]
const double BALLAST_MASS = 0; // Mass of the ballast [slugs]
const double BURNOUT_MASS = 34.959/32.17; // Mass of the rocket with burned out motor [slugs]
const double M = BURNOUT_MASS + BALLAST_MASS; // Total mass of the system
const double CG = 70.61/12.0; // distance from the tip of the nosecone to the CG [ft]
const double A = 20.831/144.0; // Cross sectional area of the rocket airframe [ft^2]
const Eigen::Vector3d G(0,0,-32.17); // in ft/s^2

//
// Electronics constants
//
const double DT = 0.01; // time step for integration

//
// Atmospheric constants
//
const int P0 = 0; // Gauge pressure on the ground [inHg]
const int T0 = 0; // Temperature on the ground [F]
const int wind_velocity = 0; //Average wind velocity
const int wind_angle = 0; //Average wind angle
const double alpha = 5/3; //Pink noise 1/f power
const double wind_std = 2.252;

//
// Controls function: 
// Control function for ADS actuation, ramps from F=0 at t=0 to F=1 at t=a (a is defined by the function)
//
double F(double t); 

#endif 