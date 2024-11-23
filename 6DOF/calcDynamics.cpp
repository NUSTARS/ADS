/*calcDynamics.cpp*/

// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <random>
#include "q.h"
#include "calcDynamics.h"
#include "constants.h"
#include "aeroData.h"
#include "cmath"

using std::random_device;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using std::sqrt;



double getV_Squared(q curr_q){
    double vx = curr_q.getV()(0);
    double vy = curr_q.getV()(1);
    double vz = curr_q.getV()(2);
    double v_mag = vx*vx + vy*vy + vz*vz;
    return v_mag;
};


double getAlpha(q curr_q){
    double vx = curr_q.getV()(0);
    double vy = curr_q.getV()(1);
    double vz = curr_q.getV()(2);
    double alpha = acos(vx/sqrt(getV_Squared(curr_q))); 
    return alpha;
};


Vector3d getAeroForces(q curr_q) {
    //v_squared != vmag because we need wind; FIX
    // need to add some sort of wind term 

    double vx = curr_q.getV()(0);
    double vy = curr_q.getV()(1);
    double vz = curr_q.getV()(2);
    double h = curr_q.getH();
    double u = curr_q.getU();
    double v_mag = getV_Squared(curr_q);
    //maybe add a new function here that says calc wind --> need to look @ pink noise model

    double FAx = 0.0;
    double FAy = 0.0;
    double FAz = 0.0;

    if(v_mag>0.0){ 
        double alpha = getAlpha(curr_q);
        
        double FAx = -0.5*A*getRho(h)*getCD(v_mag, alpha, u, h)*v_mag;
        double FAn = 0.5*A*getRho(h)*getCN(v_mag, alpha, u, h)*v_mag;

        if(sqrt(vy*vy+vz*vz) > 0){ //check this to make sure it makes sense dynamics wise
            double FAy = -FAn*vy / sqrt(vy*vy+vz*vz);
            double FAz = -FAn*vz / sqrt(vy*vy+vz*vz);
        }
    }

    Vector3d v(FAx,FAy,FAz);
    return v;
};

Vector3d getAeroMoments(q curr_q) {
    Vector3d forces = getAeroForces(curr_q);
    //just gonna assume roll = 0
    double dist; //this is the axial distance between Cg and Cp - the moment arm
    dist = getCP(getV_Squared(curr_q), getAlpha(curr_q), curr_q.getU(), curr_q.getH()) - CG;
    Vector3d v(0, dist * forces(2), dist * forces(1));
    return v;
};

// given coords in body frame, converts to earth frame
Matrix3d getR(q curr_q) { 
    Matrix3d m(3,3);
    double phi = curr_q.getTheta()(0);
    double theta = curr_q.getTheta()(1);
    double psi = curr_q.getTheta()(2);
    
    Matrix3d Rx {{1,         0,         0},
                 {0,  cos(theta),  sin(theta)},
                 {0, -sin(theta), cos(theta)}};

    Matrix3d Ry {{cos(phi), 0, -sin(phi)},
                 {0         , 1,           0},
                 {sin(phi), 0,  cos(phi)}};
    Matrix3d Rz {{cos(psi) , sin(psi), 0},
                 {-sin(psi), cos(psi), 0},
                 {        0,        0, 1}};
    Matrix3d R_SB {{0, 0, 1}, {0, -1, 0}, {1, 0, 0}};
    
    m = R_SB * Rx * Ry * Rz;

    return m;
};

// given coords in earth frame, converts to body frame 
Matrix3d getRinv(q curr_q){

    /*
    double phi = curr_q.getTheta()(0);
    double theta = curr_q.getTheta()(1);
    double psi = curr_q.getTheta()(2);

    //My brain tells me this won't work but this is what matlab says
    Matrix3d R   {{1, sin(phi)*tan(theta), cos(phi)*tan(theta)}, 
                  {0, cos(phi), -sin(phi)},
                  {0, sin(phi)/cos(theta), cos(phi)/cos(theta)}};
    */
    Matrix3d R = getR(curr_q).inverse();
    return R;
    
}
    
//calculates pink wind noise signal and multiplies by wind direction vector to get wind noise vector (idk math so hopefully u understand what i mean)
Vector3d calcWindNoise(q curr_q, Eigen::Vector2d* old_w) {
    //generate pseudorandom number (wind)
    Eigen::Vector2d past_w = *old_w;
    std::random_device rd;
    double w = rd();
    //calculate noise using pink noise formula, normalizing by wind std
    double a0 = 0;
    double a1 = a(1, a0);
    double a2 = a(2, a1);
    double pinkNoise = w - a1 * (past_w(0)) - a2* past_w(1);
    double windNoise = wind_velocity + pinkNoise/wind_std;

    past_w(1) = past_w(0);
    past_w(0) = w;
    old_w = &past_w;

    Vector3d ang(0,sin(wind_angle),cos(wind_angle)); //i am assuming wind is planar (max said this was ok)
    // return windNoise * ang;
    return Eigen::Vector3d(0,0,0);
};

double a(int k, double past_A) {
    return (k - 1 - alpha/2) * (past_A/k);
}
