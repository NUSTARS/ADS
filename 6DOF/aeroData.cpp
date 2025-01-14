#include "aeroData.h"
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
    return (0.48+u/2.0) + (vSquared*1.15E-07); // TODO get this from CFD ppl
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
    return (pow(vSquared,1.27/2.0)*0.00025 + 85.7)/12; // TEMP
}

//
// getRho
//
// returns the air density [slug/ft^3] at the current altitude
//
double getRho(double h){
    double rho = -1;
    if ( h < 36152) {
        double T = 59 - 0.00356 * h;
        double p = 2116 * pow((T + 459.7) / 518.6, 5.256);
        rho = p / (1718 * (T + 459.7)) ;
    }
    else if(h < 82345){
        double T = -70;
        double p = 473.1 * exp(1.73 - 0.000048 * h);
        rho = p / (1718 * (T + 459.7));
    }

    return rho; // TODO get this from CFD ppl
}