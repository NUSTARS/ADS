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
#include <fstream>
#include <Eigen/Dense>
#include "q.h"
#include "calcDynamics.h"
#include "aeroData.h"
#include "constants.h"
#include "simulator.h"
#include <cmath>




q getqdot(q curr_q){

    Eigen::Vector3d v = curr_q.getV();
    Eigen::Vector3d omega = curr_q.getOmega();

    Eigen::Vector3d F_aero = getAeroForces(curr_q); 
    Eigen::Vector3d M_aero = getAeroMoments(curr_q);
    Eigen::Vector3d F_grav = M*getRinv(curr_q)*G;
    
    //std::cout<<F_aero.cross(v)<<std::endl;
    //std::cout<<F_aero<<std::endl;

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

q integrate(q curr_q, Eigen::Vector2d* old_w){

    Eigen::Vector3d windNoise = calcWindNoise(curr_q, old_w);

    q k1 = getqdot(curr_q) * DT;
    q k2 = getqdot(curr_q + k1/2.0) * DT;
    q k3 = getqdot(curr_q + k2/2.0) * DT;
    q k4 = getqdot(curr_q + k3) * DT;
    q new_q = curr_q + (k1 + k2*2.0 + k3*2.0 + k4) * (1/6.0);

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
    double t = 0.0;
    std::vector<double> times;
	std::vector<double> alts;
	std::vector<double> velocx;
	std::vector<double> velocy;
	std::vector<double> velocz;
	std::vector<double> omegax;
	std::vector<double> omegay;
	std::vector<double> omegaz;
	std::vector<double> thetax;
	std::vector<double> thetay;
	std::vector<double> thetaz;
	times.push_back(t);
        alts.push_back(temp_q.getH());
        velocx.push_back(temp_q.getV()(0));
        velocy.push_back(temp_q.getV()(1));
        velocz.push_back(temp_q.getV()(2));
        omegax.push_back(temp_q.getOmega()(0));
        omegay.push_back(temp_q.getOmega()(1));
        omegaz.push_back(temp_q.getOmega()(2));
        thetax.push_back(temp_q.getTheta()(0));
        thetay.push_back(temp_q.getTheta()(1));
        thetaz.push_back(temp_q.getTheta()(2));
    Eigen::Vector2d* old_w = new Eigen::Vector2d(0,0);

    while(!atApogee(temp_q)){
        //temp_q.setU(F(t-b)); remove control for now
        temp_q.setU(0);
        temp_q = integrate(temp_q, old_w);
        t += DT;
        times.push_back(t);
        alts.push_back(temp_q.getH());
        velocx.push_back(temp_q.getV()(0));
        velocy.push_back(temp_q.getV()(1));
        velocz.push_back(temp_q.getV()(2));
        omegax.push_back(temp_q.getOmega()(0));
        omegay.push_back(temp_q.getOmega()(1));
        omegaz.push_back(temp_q.getOmega()(2));
        thetax.push_back(temp_q.getTheta()(0));
        thetay.push_back(temp_q.getTheta()(1));
        thetaz.push_back(temp_q.getTheta()(2));
    
    }
    
    std::ofstream outfile("data.csv");
	

	
	// Write data rows
    for (size_t i = 0; i < alts.size(); ++i) {
        outfile << times[i] << "," << alts[i] << "," << velocx[i] << "," << velocy[i] << "," << velocz[i] << "," << omegax[i] << "," << omegay[i] << "," << omegaz[i] << "," << thetax[i] << "," << thetay[i] << "," << thetaz[i] << "\n";
    }

    outfile.close();


    return temp_q.getH();

}

double getApogee(q curr_q){
    return getApogee(curr_q, 1000);// shifting 1000 seconds to the right is plenty for a 20s flight time to make sure we never hit
}

q  getqdot_testing(q curr_q){

    Eigen::Vector3d v = curr_q.getV();
    Eigen::Vector3d omega = curr_q.getOmega();

    Eigen::Vector3d F_aero = getAeroForces(curr_q); 
    Eigen::Vector3d M_aero = getAeroMoments(curr_q);
    Eigen::Vector3d F_grav = M*getR(curr_q).inverse()*G;

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

    //------------------------------------------------------------------

    Eigen::Vector3d world_v_vec = getR(curr_q)*v;
    Eigen::Vector3d world_a_vec = getR(curr_q)*vdot;
    Eigen::Vector3d world_lat_v_vec(world_v_vec(0), world_v_vec(1), 0);
    Eigen::Vector3d world_lat_a_vec(world_a_vec(0), world_a_vec(1), 0);

    double vert_v = world_v_vec(2);
    double vert_a = world_a_vec(2);
    double lat_v = world_lat_v_vec.norm();
    double lat_a = world_lat_a_vec.norm();
    double total_v = v.norm();
    double total_a = vdot.norm();
    double aoa = getAlpha(curr_q);
    //double drag = F_aero.norm();
    double drag = (F_aero)(0);
    //double drag = F_aero.dot(v)/v.norm(); // drag projected onto v 

    std::cout<< "grav:" << F_grav << std::endl;
    std::cout<< "aero:" << F_aero << std::endl;

    std::cout << "| alt: " << curr_q.getH();
    std::cout << "| vert_v: " << vert_v;
    std::cout << "| vert_a: " << vert_a;
    std::cout << "| tot_v: " << total_v;
    std::cout << "| tot_a: " << total_a;
    std::cout << "| lat_v: " << lat_v;
    std::cout << "| lat_a: " << lat_a;
    std::cout << std::endl;
    std::cout << "| aoa: " << (180.0/M_PI)*aoa; // deg
    std::cout << "| roll rate: " << omega(0)/(2.0*M_PI); // r/s
    std::cout << "| pitch rate: " << omega(1)/(2.0*M_PI); // r/s
    std::cout << "| yaw rate: " << omega(2)/(2.0*M_PI); // r/s
    std::cout << "| drag: " << 4.448*drag; // N
    std::cout << "| zenith: " << 90 - (180.0/M_PI)*euler(2); // deg
    std::cout << "| azimuth: " << (180.0/M_PI)*euler(0); // deg
    std::cout << std::endl;



    //------------------------------------------------------------------

    //udot always 0 since we control it so the dynamics don't update it
    return q(vdot, omegadot, thetadot, hdot, 0.0);

}

double getApogee_testing(q curr_q){
    q temp_q = curr_q;
    double t = 0.0;
    char dummy = 0;
    Eigen::Vector2d* old_w = new Eigen::Vector2d(0,0);
    temp_q.setU(0);

    while(!atApogee(temp_q)){
        std::cout << "TIME: " << t << std::endl;
        std::cout << temp_q << std::endl;
        getqdot_testing(temp_q);
        std::cout << "PRESS ENTER TO ADVANCE TO THE NEXT TIME STEP." << std::endl; 
        std::cin.get();
        temp_q = integrate(temp_q, old_w);
        t += DT;
    }

    return temp_q.getH();

}