#include "aeroData.h"
#include "constants.h"
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

    double alphaDeg = alpha * 180.0/M_PI;
    double v = sqrt(vSquared);
    int u_per = 100*u;


    return 2/(getRho(0)*vSquared*A)*(0.353757 - 0.006119 * v - 0.074949 * alphaDeg + 0.001801 * u  + 0.000089 * vSquared 
            + 0.000528 * v * alphaDeg
            + 0.000170 * v * u_per
            + 0.010095 * alphaDeg*alphaDeg
            + 0.000036 * alphaDeg * u_per
            - 0.000133 * u_per*u_per);
}

//
// getCN
//
// Takes in vSquared, angle of attack, control signal, and altitude to output the current Cn
//
double getCN(double vSquared, double alpha, double u, double h){

    double alphaDeg = alpha * 180.0/M_PI;
    double v = sqrt(vSquared);
    int u_per = 100*u;


    return 2/(getRho(0)*vSquared*A)*(-0.054285 - 
            0.002019 * v - 
            0.140628 * alphaDeg - 
            0.000810 * u_per + 
            0.000020 * vSquared + 
            0.000947 * v * alphaDeg - 
            0.000001 * v * u_per +
            0.015547 * alphaDeg*alphaDeg + 0.000030 * alphaDeg * u_per + 0.000007 * u_per*u_per);
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
    int u_per = 100*u;


    return 81.63465778
     - 0.00866002 * u_per
     + 0.00007982 * u_per*u_per
     + 0.02379748 * v
     + 0.00019875 * u_per * v
     - 0.00000093 * u_per*u_per * v
     - 0.00006348 * vSquared
     - 0.00000056 * u_per * vSquared
     + 0.00000000 * u_per*u_per* vSquared;
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