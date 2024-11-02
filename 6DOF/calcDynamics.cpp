/*calcDynamics.cpp*/

// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025

#include <iostream>
#include <Eigen/Core>
#include "q.h"
#include "calcDynamics.h"
#include "constants.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using std::sqrt;

double getAlpha(q){
    double vx = q.getVX();
    double vy = q.getVY();
    double vz = q.getVZ();
    double alpha = vx/sqrt(vx*vx + vy*vy + vz*vz)
    return alpha;
};

Vector3d getAeroForces(q) {
    //v_squared != vmag because we need wind; FIX
    // need to add some sort of wind term 

    //maybe add a new function here that says calc wind --> need to look @ pink noise model 
    double vx = q.getVX();
    double vy = q.getVY();
    double vz = q.getVZ();
    double h = q.getH();
    double u = q.getU();
    double alpha = getAlpha();
    double v_mag = vx*vx + vy*vy + vz*vz
    
    double FAx = 0.5*A*getRho(h)*getCD(v_mag, alpha, u, h)*v_mag
    double FAn = 0.5*A*getRho(h)*getCN(v_mag, alpha, u, h)*v_mag
    double FAy = -FAn*vy / sqrt(vy*vy+vz*vz)
    double FAz = -FAn*vz / sqrt(vy*vy+vz*vz)

    Vector3d v(FAx,FAy,FAz);
    return v;
};

Vector3d getAeroMoments() {
    //place holder code -- need to fill in 
    Vector3d v(1,2,3);
    return v;
};


Matrix3d getRinv() {
    Matrix3d m(3,3);
   //fill in the rest of this matrix using q
   return m;
};
    

Vector3D calcWindNoise() {
    ///FILL IN WITH RANDOM MODEL probably a function of something 
    Vector3D w(1,2,3);
    return w;
}