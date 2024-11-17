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
#include <Eigen/Dense>
#include "q.h"
#include "calcDynamics.h"
#include "constants.h"



q getqdot(q curr_q){

    Eigen::Vector3d v = curr_q.getV();
    Eigen::Vector3d omega = curr_q.getOmega();

    Eigen::Vector3d F_aero = getAeroForces(curr_q);
    Eigen::Vector3d M_aero = getAeroMoments(curr_q);
    Eigen::Vector3d F_grav = getR(curr_q).inverse()*G;

    double vXdot = (1/M)*(F_aero(0) + F_grav(0)) - (omega(1)*v(2)-omega(2)*v(1));
    double vYdot = (1/M)*(F_aero(1) + F_grav(1)) - (omega(2)*v(0)-omega(0)*v(2));
    double vZdot = (1/M)*(F_aero(2) + F_grav(2)) - (omega(0)*v(1)-omega(1)*v(0));

    double omegaXdot = (1/Ix)*(M_aero(0) - v(1)*v(2)*(Iz-Iy));
    double omegaYdot = (1/Iy)*(M_aero(1) - v(0)*v(2)*(Ix-Iz));
    double omegaZdot = (1/Iz)*(M_aero(2) - v(0)*v(1)*(Iy-Ix));

    Eigen::Vector3d vdot(vXdot, vYdot, vZdot);
    Eigen::Vector3d omegadot(omegaXdot, omegaYdot, omegaZdot); 
    Eigen::Vector3d thetadot = curr_q.getOmega(); // FIX
    double hdot = (getR(curr_q)*curr_q.getV())(2);

    //udot always 0 since we control it so the dynamics don't update it
    return q(vdot, omegadot, thetadot, hdot, 0.0);

}

q integrate(q curr_q, Eigen::Vector2d* old_w){

    Eigen::Vector3d windNoise = calcWindNoise(curr_q, old_w);

    q k1 = getqdot(curr_q) * DT;
    //q k2 = getqdot(curr_q + k1/2.0) * DT;
    //q k3 = getqdot(curr_q + k2/2.0) * DT;
    //q k4 = getqdot(curr_q + k3) * DT;
    //q new_q = curr_q + (k1 + k2*2.0 + k3*2.0 + k4) * (1/6.0);
    q new_q = curr_q + k1*DT;
    return new_q;
}


bool atApogee(q curr_q){
    return (getR(curr_q)*curr_q.getV())(2) < 0;
}

double getApogee(q curr_q, double b){
    q temp_q = curr_q;
    double t = 0.0;
    Eigen::Vector2d* old_w = new Eigen::Vector2d(0,0);

    while(!atApogee(temp_q)){
        //temp_q.setU(F(t-b));
        temp_q.setU(0);
        temp_q = integrate(temp_q, old_w);
        t += DT;
    }
    
    //gp << "set xlabel 'time'\n";
    //gp << "set ylabel 'Height'\n";
    //gp << "set key top left\n"; // Position the legend

    // Plot one dataset per axis
    //gp << "plot '-' with lines title 'Height' lt rgb 'blue'\n";

    // Send the datasets to gnuplot
    //gp.send1d(height); // Data for the first y-axis

    

    return temp_q.getH();
}

double getApogee(q curr_q){
    return getApogee(curr_q, 1000);// shifting 1000 seconds to the right is plenty for a 20s flight time to make sure we never hit
}