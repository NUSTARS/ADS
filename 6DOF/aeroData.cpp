#include "aeroData.h"
#include <cmath>
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
    return cos(alpha)*(0.5+u/2.0); // TODO get this from CFD ppl
}

//
// getCN
//
// Takes in vSquared, angle of attack, control signal, and altitude to output the current Cn
//
double getCN(double vSquared, double alpha, double u, double h){
    return sin(alpha)*(0.5+u/2.0); // TODO get this from CFD ppl
}


//
// getCP
//
// Takes in vSquared, angle of attack, control signal, and altitude to output 
// the current distance [ft] from the CP to tip of the nosecone
//
double getCP(double vSquared, double alpha, double u, double h){
    return 83.0/12; // TODO get this from CFD ppl
}

//
// getRho
//
// returns the air density [lb/ft^3] at the current altitude
//
double getRho(double h){
    return -1; // TODO get this from CFD ppl
}