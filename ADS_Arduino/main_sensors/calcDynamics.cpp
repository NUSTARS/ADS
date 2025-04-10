/*calcDynamics.cpp*/

// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025

// #include <iostream>
//#include <Eigen/Core>
//#include <Eigen/Dense>
#include <ArduinoEigenDense.h>
#include <ArduinoEigen.h>
#include "q.h"
#include "calcDynamics.h"
#include "constants.h"
#include "aeroData.h"
#include <cmath>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using std::sqrt;



double getV_Mag(q curr_q){
    double vx = curr_q.getV()(0);
    double vy = curr_q.getV()(1);
    double vz = curr_q.getV()(2);
    double v_sqr = vx*vx + vy*vy + vz*vz;
    return sqrt(v_sqr);
};


double getAlpha(q curr_q){
    double vx = curr_q.getV()(0);
    double vy = curr_q.getV()(1);
    double vz = curr_q.getV()(2);

    double alpha = atan2(sqrt(vy*vy + vz*vz), vx);
    
    return alpha;
};


Vector3d getAeroForces(q curr_q) {
    // need to add some sort of wind term 

    double vx = curr_q.getV()(0);
    double vy = curr_q.getV()(1);
    double vz = curr_q.getV()(2);
    double h = curr_q.getH();
    double u = curr_q.getU();
    double v_squared = vx*vx+vy*vy+vz*vz;
    double v_mag = sqrt(vy*vy+vz*vz);
 
    double FAx = 0.0;
    double FAy = 0.0;
    double FAz = 0.0;

    if(v_squared > 0.0){ 
        double alpha = getAlpha(curr_q);
        
        FAx = -0.5*A*getRho(h)*getCD(v_squared, alpha, u, h)*v_squared;
        double FAn = 0.5*A*getRho(h)*getCN(v_squared, alpha, u, h)*v_squared;

        if(v_mag > 0){ // should already by true if FAn > 0
            FAy = FAn*vy / v_mag;
            FAz = -FAn*vz / v_mag;
        }
    }

    Vector3d v(FAx,FAy,FAz);
    // Vector3d v(FAx,0,0);
    return v;
};


Vector3d getAeroMoments(q curr_q, Vector3d forces) {
    double vx = curr_q.getV()(0);
    double vy = curr_q.getV()(1);
    double vz = curr_q.getV()(2);
    // Vector3d forces = getAeroForces(curr_q);
    //just gonna assume roll = 0
    double dist; //this is the axial distance between Cg and Cp - the moment arm
    dist = getCP(vx*vx +vy*vy +vz*vz, getAlpha(curr_q), curr_q.getU(), curr_q.getH()) - CG;
    Vector3d v(0, dist * forces(2), dist * forces(1));
    // Vector3d v(0,0,0);
    return v;
};

Matrix3d getR(q curr_q) { 
    double phi = curr_q.getTheta()(0);
    double theta = curr_q.getTheta()(1);
    double psi = curr_q.getTheta()(2);

    Matrix3d m {{-sin(theta) , sin(phi)*cos(theta), cos(phi)*cos(theta)},
                 {-sin(psi)*cos(theta), -sin(phi)*sin(psi)*sin(theta)-cos(phi)*cos(psi), sin(phi)*cos(psi)-sin(psi)*sin(theta)*cos(phi)},
                 {cos(psi)*cos(theta), sin(phi)*sin(theta)*cos(psi)-sin(psi)*cos(phi), sin(phi)*sin(psi)+sin(theta)*cos(phi)*cos(psi)}};

    return m;
}

// given coords in earth frame, converts to body frame 
Matrix3d getRinv(q curr_q){
    double phi = curr_q.getTheta()(0);
    double theta = curr_q.getTheta()(1);
    double psi = curr_q.getTheta()(2);


    Matrix3d R_inv {{-sin(theta), -sin(psi)*cos(theta), cos(psi)*cos(theta)},
                  {sin(phi)*cos(theta), -sin(phi)*sin(psi)*sin(theta)-cos(phi)*cos(psi), sin(phi)*sin(theta)*cos(psi)-sin(psi)*cos(phi)},
                  {cos(phi)*cos(theta), sin(phi)*cos(psi) - sin(psi)*sin(theta)*cos(phi), sin(psi)*sin(phi) + sin(theta)*cos(psi)*cos(phi)}};

    return R_inv;
    
};