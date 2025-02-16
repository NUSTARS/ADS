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
const double BALLAST_MASS = 0.0; // Mass of the ballast [slugs]

const double Ix = 0.04979; // Moment of inertia about the X-axis [lbf*ft*s^2]
const double Iy = 6.81; // Moment of inertia about the Y-axis [lbf*ft*s^2]
//const double Iy = 1;
const double Iz = Iy; // Moment of inertia about the Z-axis [lbf*ft*s^2]
const double BURNOUT_MASS = 36.727/32.174; // Mass of the rocket with burned out motor [slugs]
const double M = BURNOUT_MASS + BALLAST_MASS; // Total mass of the system [slug]
const double CG = 68.093/12.0; // distance from the tip of the nosecone to the CG [ft]
const double A = 20.831/144.0; // Cross sectional area of the rocket airframe [ft^2]
const Eigen::Vector3d G(0,0,-32.155); // [ft/s^2]
const double PAD_ALT = 700; // [ft] 

//
// Electronics constants
//
const double DT = 0.05; // time step for integration

//
// Atmospheric constants
//
const double P0 = 988.227*100; // Gauge pressure on the ground [Pa]
const double T0 = 13.613; // Temperature on the ground [C]
// const double WIND_VELOCITY = 20.0 * 1.46667; //Average wind velocity [ft/s]
const double WIND_VELOCITY = 10.0 * 1.46667;
const double WIND_ANGLE = 0.0; //Average wind angle
const double WIND_STD = 4.0;
const double ALPHA = 5/3; //Pink noise 1/f power

const double IMU_NOISE_MEAN = 0.0;
const double IMU_NOISE_STDDEV = 0.01;
const double BMP_NOISE_MEAN = 0.0;
const double BMP_NOISE_STDDEV = 1.0;

const unsigned int SEED = 2;

// Controls function: 
// Control function for ADS actuation, ramps from F=0 at t=0 to F=1 at t=a (a is defined by the function)
//
double F(double t);
const double RAMP_TIME = 1.0; // how long the function takes to ramp 0 to full actuation [s]

#endif