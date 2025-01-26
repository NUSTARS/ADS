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
    return (0.481+u/2.0) + (vSquared*1.15E-07); // TODO get this from CFD ppl
}

//
// getCN
//
// Takes in vSquared, angle of attack, control signal, and altitude to output the current Cn
//
double getCN(double vSquared, double alpha, double u, double h){
    return pow(alpha,1.25)*(25+u/2.0); // TODO get this from CFD ppl
}


//
// getCP
//
// Takes in vSquared, angle of attack, control signal, and altitude to output 
// the current distance [ft] from the CP to tip of the nosecone
//
double getCP(double vSquared, double alpha, double u, double h){

    double x = sqrt(vSquared); // x is the square root of vsquared

    return 63.5 + 0.526 * x + (-5.28E-03) * pow(x, 2) + 2.95E-05 * pow(x, 3) + (-9.95E-08) * pow(x, 4) + 
           2.08E-10 * pow(x, 5) + (-2.63E-13) * pow(x, 6) + 1.84E-16 * pow(x, 7) + (-5.49E-20) * pow(x, 8);

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