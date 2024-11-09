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
q q::operator+(const q& obj){
    Eigen::Vector3d newV = v + obj.getV();
    Eigen::Vector3d newOmega = omega + obj.getOmega();
    Eigen::Vector3d newTheta = theta + obj.getTheta();
    double newH = h + obj.getH();
    double newU = u + obj.getU();

    return q(newV, newOmega, newTheta, newH, newU);
}
q q::operator*(double a){
    Eigen::Vector3d newV = a*v;
    Eigen::Vector3d newOmega = a*omega;
    Eigen::Vector3d newTheta = a*theta;
    double newH = a*h;
    double newU = a*u;

    return q(newV, newOmega, newTheta, newH, newU);
}
q operator*(double a, q& obj){
    return obj*a;
}
q q::operator/(double a){
    return (*this)*(1.0/a);
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