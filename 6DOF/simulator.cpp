/*simulator.cpp*/
//
// preforms all of the dynamicds simulation for integration
//
//

// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025

#include <iostream>
#include <Eigen/Core>
#include "q.h"
#include "calcDynamics.h"
#include "constants.h"

q getqdot(q curr_q){
    Eigen::Vector3d v = curr_q.getV();
    Eigen::Vector3d omega = curr_q.getOmega();

    Eigen::Vector3d F_aero = getAeroForces(curr_q);
    Eigen::Vector3d M_aero = getAeroMoments(F_aero, curr_q);
    Eigen::Vector3d F_grav = getR()*G;

    double vXdot = (1/M)*(F_aero(0) + F_grav(0)) - (omega(1)*v(2)-omega(2)*v(1));
    double vYdot = (1/M)*(F_aero(1) + F_grav(1)) - (omega(2)*v(0)-omega(0)*v(2));
    double vZdot = (1/M)*(F_aero(2) + F_grav(2)) - (omega(0)*v(1)-omega(1)*v(0));

    double omegaXdot = (1/Ix)*(M_aero(0) - v(1)*v(2)*(Iz-Iy));
    double omegaYdot = (1/Iy)*(M_aero(1) - v(0)*v(2)*(Ix-Iz));
    double omegaZdot = (1/Iz)*(M_aero(2) - v(0)*v(1)*(Iy-Ix));

    Eigen::Vector3d vdot(vXdot, vYdot, vZdot);
    Eigen::Vector3d omegadot(omegaXdot, omegaYdot, omegaZdot); 
    Eigen::Vector3d thetadot = curr_q.getOmega();
    double hdot = (getR()*curr_q.getV())(2);

    //udot always 0 since we control it so the dynamics don't update it
    return q(vdot, omegadot, thetadot, hdot, 0.0);
}

q integrate(q curr_q){
    // Only post multiplication works right now, should fix
    q k1 = getqdot(curr_q) * DT;
    q k2 = getqdot(curr_q + k1/2.0) * DT;
    q k3 = getqdot(curr_q + k2/2.0) * DT;
    q k4 = getqdot(curr_q + k3) * DT;
    q new_q = curr_q + (k1 + k2*2.0 + k3*2.0 + k4) * (1/6.0);
    return new_q;
}


bool atApogee(q curr_q){
    return (getR()*curr_q.getV())(2) <= 0;
}

double getApogee(q curr_q, double b){
    q temp_q = curr_q;
    double t = 0.0;
    while(!atApogee(temp_q)){
        temp_q.setU(F(t-b));
        temp_q = integrate(temp_q);
        t += DT;
    }
}

double getApogee(q curr_q){
    getApogee(curr_q, 1000);// shifting 1000 seconds to the right is plenty for a 20s flight time to make sure we never hit
}