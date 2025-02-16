#include "aeroData.h"
#include "constants.h"
#include <cmath>
#include "constants.h"
/*aeroData.cpp*/

//
// Lookup tables for aerodnyamic coefficents based on current state and atmospheric conditions
//
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025
//

//
// getCD
//
// Takes in vSquared, angle of attack, control signal, and altitude to output the current Cd
//
double getCD(double vSquared, double alpha, double u, double h){
    //return (0.481+u/2.0) + (vSquared*1.15E-07); // TODO get this from CFD ppl

    double alphaDeg = alpha * 3.1415/180.0;
    double v = sqrt(vSquared);
    
    return (-0.0021716*alphaDeg - 6.4358e-06*v + 1.8701e-06*v*alpha + 1.2254e-07*vSquared + 0.0020791*pow(alpha,2) + 0.48279) + u/5.0;
}

//
// getCN
//
// Takes in vSquared, angle of attack, control signal, and altitude to output the current Cn
//
double getCN(double vSquared, double alpha, double u, double h){

    double alphaDeg = alpha * 3.1415/180.0;
    double v = sqrt(vSquared);

    return 0.18966*alphaDeg - 2.7885e-06*v + 1.7577e-05*alphaDeg*v + 0.00051704 + 3.4281e-09*vSquared + 0.010233*pow(alphaDeg,2);

    //return pow(alpha,1.25)*(25+u/2.0); // TODO get this from CFD ppl
}


//
// getCP
//
// Takes in vSquared, angle of attack, control signal, and altitude to output 
// the current distance [ft] from the CP to tip of the nosecone
//
double getCP(double vSquared, double alpha, double u, double h){

    double alphaDeg = alpha * 3.1415/180.0;
    double v = sqrt(vSquared);

    return -1.2251*alphaDeg - 0.000270051*v + 0.00015956*alphaDeg*v + 85.9529 + 2.10647e-06*vSquared + 0.0692183*pow(alphaDeg,2);

    // return (pow(vSquared,1.27/2.0)*0.00025 + 85.7)/12.0; // TEMP
}

//
// getRho
//
// returns the air density [slug/ft^3] at the current altitude
//
double getRho(double h){

    double tKelvin = T0 + 273.15; // [K]
    double currentAlt = (h+PAD_ALT)/3.281; // [m]
    
    double pressure = P0*exp(-0.0341688*currentAlt/tKelvin); // [Pa]
    double temperature = tKelvin - 0.0065*0.3048*h; // [K]

    double rho = pressure/(287.050*temperature); // [kg/m^3]

    rho *= 0.00194032; // [slugs/ft^3]

    return rho; // TODO get this from CFD ppl
}