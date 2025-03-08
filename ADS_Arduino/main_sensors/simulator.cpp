/*simulator.cpp*/
//
// preforms all of the dynamicds simulation for integration
//
//

// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025

// #include <iostream>
#include <ArduinoEigen.h>
// #include <fstream>
#include <ArduinoEigenDense.h>
#include "q.h"
#include "calcDynamics.h"
#include "aeroData.h"
#include "constants.h"
#include "simulator.h"
#include "wind.h"
#include <cmath>




q getqdot(q curr_q, Wind* wind){

    // std::cout << "function started !!" << std::endl;

    //The local wind velocity is added to the rocket velocity to get the airspeed velocity of the rocket. 
    //By inverse rotation this quantity is obtained in rocket coordinates, from which the angle of
    //attack and other flight parameters can be computed
    Eigen::Vector3d v = curr_q.getV();
    Eigen::Vector3d omega = curr_q.getOmega();
    Eigen::Vector3d wind_3d_world(wind->getWind()(0), wind->getWind()(1), 0.0);
    // std::cout << wind_3d_world << std::endl;
    // Eigen::Vector3d wind_3d_body(1, 1,1);

    Eigen::Vector3d wind_body = getRinv(q(v, omega, curr_q.getTheta(), curr_q.getH(), curr_q.getU()))*wind_3d_world;

    // std::cout << wind_3d_body << std::endl;
    // std::cout << wind_body << std::endl;

    // Eigen::Vector3d wind_body(1, 1,1);
    //Add local wind to the rocket velocity 

    Eigen::Vector3d v_new(curr_q.getV()(0) + wind_body(0), curr_q.getV()(1) + wind_body(1),curr_q.getV()(2) + wind_body(2));
    
    // std::cout << v_new << std::endl;


    q new_q_wind(v_new, omega, curr_q.getTheta(), curr_q.getH(), curr_q.getU());


    Eigen::Vector3d F_aero = getAeroForces(new_q_wind); 
    Eigen::Vector3d M_aero = getAeroMoments(new_q_wind, F_aero);
    Eigen::Vector3d F_grav = M*getRinv(curr_q)*G;

    double vXdot = (1/M)*(F_aero(0) + F_grav(0)) - (omega(1)*v(2)-omega(2)*v(1));
    double vYdot = (1/M)*(F_aero(1) + F_grav(1)) - (omega(2)*v(0)-omega(0)*v(2));
    double vZdot = (1/M)*(F_aero(2) + F_grav(2)) - (omega(0)*v(1)-omega(1)*v(0));

    double omegaXdot = (1/Ix)*(M_aero(0) - omega(1)*omega(2)*(Iz-Iy));
    double omegaYdot = (1/Iy)*(M_aero(1) - omega(0)*omega(2)*(Ix-Iz));
    double omegaZdot = (1/Iz)*(M_aero(2) - omega(0)*omega(1)*(Iy-Ix));

    Eigen::Vector3d euler = curr_q.getTheta();
    double phi = euler(0);
    double theta = euler(1);
    
    Eigen::Matrix3d specialR{{1, sin(phi)*sin(theta), cos(phi)*tan(theta)},
                   {0, cos(phi), -sin(phi)},
                   {0, sin(phi)/cos(theta), cos(phi)/cos(theta)}};

    Eigen::Vector3d vdot(vXdot, vYdot, vZdot);
    Eigen::Vector3d omegadot(omegaXdot, omegaYdot, omegaZdot); 
    Eigen::Vector3d thetadot = specialR*omega; 
    double hdot = (getR(curr_q)*v)(2);

    //udot always 0 since we control it so the dynamics don't update it
    return q(vdot, omegadot, thetadot, hdot, 0.0);

}

q integrate(q curr_q, Wind* wind){

    q k1 = getqdot(curr_q, wind) * DT;
    q k2 = getqdot(curr_q + k1/2.0, wind) * DT;
    q k3 = getqdot(curr_q + k2/2.0, wind) * DT;
    q k4 = getqdot(curr_q + k3, wind) * DT;
    q new_q = curr_q + (k1 + k2*2.0 + k3*2.0 + k4) * (1/6.0);
    // q new_q = curr_q;


    Eigen::Vector3d scaled_theta;
    Eigen::Vector3d theta = new_q.getTheta();
    for (int i = 0; i < theta.size(); ++i) {
        scaled_theta[i] = std::fmod(theta[i], 2*M_PI); // Wrap angle
        if (scaled_theta[i] < -M_PI) {
            scaled_theta[i] += 2*M_PI; // Ensure positivity
        }
    }

    q new_q_thetalimits (new_q.getV(), new_q.getOmega(), scaled_theta, new_q.getH(),new_q.getU());

    return new_q_thetalimits;
}


bool atApogee(q curr_q){
    return (getR(curr_q)*curr_q.getV())(2) < 0;
}

double getApogee(q curr_q, double b){
    q temp_q = curr_q;
    double time = 0.0;
    Wind wind;

    while(!atApogee(temp_q)){
        temp_q.setU(F_function(time-b)); 

        wind.updateWind();
        temp_q = integrate(temp_q, &wind);

        time = time + DT;         
    }

    return temp_q.getH();

}

double getApogee(q curr_q){
    return getApogee(curr_q, 100); // shifting 100 seconds to the right is plenty for a 20s flight time to make sure we never hit
}
