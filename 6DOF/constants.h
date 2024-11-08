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

//
// Mechanical constants
//
const double Ix = 0; // Moment of inertia about the X-axis [slugs*ft^2]
const double Iy = 0; // Moment of inertia about the Y-axis [slugs*ft^2]
const double Iz = 0; // Moment of inertia about the Z-axis [slugs*ft^2]
const double BALLAST_MASS = 0; // Mass of the ballast [slugs]
const double BURNOUT_MASS = 0.9; // Mass of the rocket with burned out motor [slugs]
const double M = BURNOUT_MASS + BALLAST_MASS; // Total mass of the system
const double CG = 0; // distance from the tip of the nosecone to the CG [in]
const int A = 0; // Cross sectional area of the rocket airframe [in^2]

//
// Electronics constants
//
const double DT = 0.01; // time step for integration

//
// Atmospheric constants
//
const int P0 = 0; // Gauge pressure on the ground [inHg]
const int T0 = 0; // Temperature on the ground [F]

//
// Controls function: 
// Control function for ADS actuation, ramps from F=0 at t=0 to F=1 at t=a (a is defined by the function)
//
double F(double t); 