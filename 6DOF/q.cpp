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
q::q(Eigen::Vector3d newV, Eigen::Vector3d newOmega, Eigen::Vector3d newTheta, double newH, double newU){
    v = newV;
    omega = newOmega;
    theta = newTheta;
    h = newH;
    u = newU;
}
        
//
// copy constructor:
//
q::q(const q& other){
    v = other.getV();
    omega = other.getOmega();
    theta = other.getTheta();
    h = other.getH();
    u = other.getU();
}


//
// Accessors
//
Eigen::Vector3d q::getV() const {return v;}
Eigen::Vector3d q::getOmega() const {return v;}
Eigen::Vector3d q::getTheta() const {return theta;}      
double q::getH() const {return h;}

double q::getU() const {return u;}

//
// Mutator (only for u since the rest of the state cannot be adusted)
//
void q::setU(double newU){u = newU;}