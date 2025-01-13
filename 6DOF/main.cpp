#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "q.h"
#include "calcDynamics.h"
#include "constants.h"
#include "simulator.h"
#include <random>
#include "controls.h"
#include "sensing.h"
#include <cmath>

int main() {

<<<<<<< HEAD
    // double OR_LATERAL_VELOCITY = 71.354; //[ft/s]
    // double OR_VERTICAL_VELOCITY = 692.409; //[ft/s]
    // double OR_PITCH_RATE = 4.77E-04; // [r/s]
    // double OR_YAW_RATE = 2.25E-05; // [r/s]
    // double OR_ZENITH = 84.109; // [deg]
    // double OR_AZIMUTH = 0.013; // [deg]
    // double initial_h = 921.966; // [ft]

    double OR_LATERAL_VELOCITY = 69.752; //[ft/s]
    double OR_VERTICAL_VELOCITY = 674.312; //[ft/s]
    double OR_PITCH_RATE = 1.86E-04; // [r/s]
    double OR_YAW_RATE = 2.32E-06; // [r/s]
    double OR_ZENITH = 84.071; // [deg]
    double OR_AZIMUTH = 0.012; // [deg]
    double initial_h = 897.358; // [ft]
=======
    double OR_LATERAL_VELOCITY = 65.963; //[ft/s]
    double OR_VERTICAL_VELOCITY = 634.357; //[ft/s]
    double OR_PITCH_RATE = -0.002; // [r/s]
    double OR_YAW_RATE = -1.10E-05; // [r/s]
    double OR_AZIMUTH = 0.009; // [deg]
    double OR_ZENITH = 84.049; // [deg]
    double initial_h = 806.814; // [ft]
>>>>>>> 900d9b87066e3854fb724f54e81cc137a94c31b8

    // From OpenRocket
    Eigen::Vector3d initial_v_world(0, -OR_LATERAL_VELOCITY, OR_VERTICAL_VELOCITY); // good
    Eigen::Vector3d initial_omega(0, OR_PITCH_RATE*2*M_PI, OR_YAW_RATE*2*M_PI); // good
    Eigen::Vector3d initial_theta(OR_AZIMUTH*M_PI/180.0, 0, (90-OR_ZENITH)*M_PI/180.0); // not sure

    // For Testing
    // Eigen::Vector3d initial_theta(0, -0.1, 0);
    // Eigen::Vector3d initial_omega(0, 0, 0);
    // Eigen::Vector3d initial_v_body(700, 0, 10);
    double phi = OR_AZIMUTH*M_PI/180.0;
    double theta = 0;
    double psi = (90-OR_ZENITH)*M_PI/180.0;

    Eigen::Matrix3d specialR{{1, sin(phi)*sin(theta), cos(phi)*tan(theta)},
                {0, cos(phi), -sin(phi)},
                {0, sin(phi)/cos(theta), cos(phi)/cos(theta)}};

    Eigen::Matrix3d specialRinv = specialR.inverse();

    //convert from WORLD to BODY using R_BW (R_WB inverse)
    Eigen::Vector3d initial_v_body = getRinv(q(Eigen::Vector3d(0,0,0), initial_omega, initial_theta, initial_h, 0))*initial_v_world; 

    Eigen::Vector3d omega_rate = specialRinv*initial_omega;
    
    // post boost initial state
    q currentState(initial_v_body, initial_omega, initial_theta, initial_h, 0);

    std::cout << currentState << std::endl;
    std::cout << getApogee(currentState)<< std::endl;
    //std::cout << getApogee_testing(currentState)<< std::endl;

    /* MAIN FOR ONCE WE GET GETAPOGEE WORKING

    Eigen::Vector2d* old_wind = new Eigen::Vector2d(0,0);

    double max_simTime = 30;
    double current_t = 0.0;

    // run while the sim is less than 30 seconds and havent reached apogee 
    while ((current_t < max_simTime) && !(atApogee(currentState))) {

        q SensorNoiseState = addSensorNoise(currentState);

        //call SHISHIR CODE to get b, set B in current state going forward
        double signal = find_u(SensorNoiseState, 5150, 1000); //forward integrate is placeholder
        std::cout<< signal << std::endl;

        SensorNoiseState.setU(signal);
        q currentState = integrate(SensorNoiseState, old_wind);

        current_t += DT;

    }

    */

};