/*calcDynamics.cpp*/

// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025

#include <iostream>
#include <Eigen/Core>
#include "q.h"
#include "calcDynamics.h"
#include "constants.h"
#include "aeroData.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using std::sqrt;



double getV_Squared(q state){
    double vx = state.getV()(0);
    double vy = state.getV()(1);
    double vz = state.getV()(2);
    double v_mag = vx*vx + vy*vy + vz*vz;
    return v_mag;
};


double getAlpha(q state){
    double vx = state.getV()(0);
    double vy = state.getV()(1);
    double vz = state.getV()(2);
    double alpha = vx/sqrt(getV_Squared(state)); //technically this is not angle of attack, no?
    return alpha;
};


Vector3d getAeroForces(q state) {
    //v_squared != vmag because we need wind; FIX
    // need to add some sort of wind term 

    //maybe add a new function here that says calc wind --> need to look @ pink noise model 
    double vx = state.getV()(0);
    double vy = state.getV()(1);
    double vz = state.getV()(2);
    double h = state.getH();
    double u = state.getU();
    double alpha = getAlpha(state);
    double v_mag = getV_Squared(state);
    
    double FAx = 0.5*A*getRho(h)*getCD(v_mag, alpha, u, h)*v_mag;
    double FAn = 0.5*A*getRho(h)*getCN(v_mag, alpha, u, h)*v_mag;
    double FAy = -FAn*vy / sqrt(vy*vy+vz*vz);
    double FAz = -FAn*vz / sqrt(vy*vy+vz*vz);

    Vector3d v(FAx,FAy,FAz);
    return v;
};

Vector3d getAeroMoments(q state) {
    Vector3d forces = getAeroForces(state);
    //just gonna assume roll = 0
    double Cg = 67.95; //in., from openrocket v14 (prob somewhat innacurate, I'll think abt it later)
    double dist; //this is the axial distance between Cg and Cp - the moment arm
    dist = getCP(getV_Squared(state), getAlpha(state), state.getU(), state.getH()) - Cg;
    Vector3d v(0, dist * forces(2), dist * forces(1));
    return v;
};


Matrix3d getR(q state) { 
    Matrix3d m(3,3);
    double phi = state.getOmega()(0);
    double theta = state.getOmega()(1);
    double psi = state.getOmega()(2);
    
    Matrix3d Rx {{1,         0,         0},
                 {0,  cos(phi),  sin(phi)},
                 {0, -sin(phi), cos(phi)}};

    Matrix3d Ry {{cos(theta), 0, -sin(theta)},
                 {0         , 1,           0},
                 {sin(theta), 0,  cos(theta)}};
    Matrix3d Rz {{cos(phi) , sin(phi), 0},
                 {-sin(phi), cos(phi), 0},
                 {        0,        0, 1}};
    
    m = Rx * Ry * Rz;

    return m;
};
    

Vector3d calcWindNoise() {
    ///FILL IN WITH RANDOM MODEL probably a function of something 
    Vector3d w(1,2,3);
    return w;
}