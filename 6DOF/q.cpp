#include "q.h"

/*q.cpp*/

//
// Wrapper class storing the current state (or derivative of the state) of the rocket
//
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025
//


//
// constructor (maybe make it take in an array instead?)
//
q::q(double newVX, double newVY, double newVZ, double newOmegaX, double newOmegaY, double newOmegaZ, double newTheta, double newPsi, double newPhi, double newH, double newU){
    vX = newVX;
    vY = newVY;
    vZ = newVZ;
    omegaX = newOmegaX;
    omegaY = newOmegaY;
    omegaZ = newOmegaZ;
    theta = newTheta;
    psi = newPsi;
    phi = newPhi;
    h = newH;
    u = newU;
}
        
//
// copy constructor:
//
q::q(const q& other){
    vX = other.getVX();
    vY = other.getVY();
    vZ = other.getVZ();
    omegaX = other.getOmegaX();
    omegaY = other.getOmegaY();
    omegaZ = other.getOmegaZ();
    theta = other.getTheta();
    psi = other.getPsi();
    phi = other.getPhi();
    h = other.getH();
    u = other.getU();
}


//
// Accessors
//
double q::getVX() const {return vX;}
double q::getVY() const {return vY;}
double q::getVZ() const {return vZ;}

double q::getOmegaX() const {return omegaX;}
double q::getOmegaY() const {return omegaY;}
double q::getOmegaZ() const {return omegaZ;}

double q::getTheta() const {return theta;}
double q::getPsi() const {return psi;}
double q::getPhi() const {return phi;}
        
double q::getH() const {return h;}

double q::getU() const {return u;}

//
// Mutator (only for u since the rest of the state cannot be adusted)
//
void q::setU(double newU){u = newU;}