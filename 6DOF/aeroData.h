/*aeroData.h*/

//
// Lookup tables for aerodnyamic coefficents based on current state and atmospheric conditions
//
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025
//

#ifndef aeroData_H
#define aeroData_H

//
// getCD
//
// Takes in vSquared, angle of attack, control signal, and altitude to output the current Cd
//
double getCD(double vSquared, double alpha, double u, double h);

//
// getCN
//
// Takes in vSquared, angle of attack, control signal, and altitude to output the current Cn
//
double getCN(double vSquared, double alpha, double u, double h);


//
// getCP
//
// Takes in vSquared, angle of attack, control signal, and altitude to output 
// the current distance [in] from the CP to tip of the nosecone
//
double getCP(double vSquared, double alpha, double u, double h);

//
// getRho
//
// returns the air density [lb/ft^3] at the current altitude
//
double getRho(double h);

#endif // aeroData_H