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
// overloading operators
//
q q::operator+(const q& obj2){
    Eigen::Vector3d newV = v + obj2.getV();
    Eigen::Vector3d newOmega = omega + obj2.getOmega();
    Eigen::Vector3d newTheta = theta + obj2.getTheta();
    double newH = h + obj2.getH();
    double newU = u + obj2.getU();

    return q(newV, newOmega, newTheta, newH, newU);
}
q operator*(const q& obj, double a){
    Eigen::Vector3d newV = a*obj.getV();
    Eigen::Vector3d newOmega = a*obj.getOmega();
    Eigen::Vector3d newTheta = a*obj.getTheta();
    double newH = a*obj.getH();
    double newU = a*obj.getU();

    return q(newV, newOmega, newTheta, newH, newU);
}
q q::operator-(const q& obj2){
    return obj1 + -1.0*obj2;
}
q q::operator/(const q& obj, double a){
    return (1.0/a)*obj;
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