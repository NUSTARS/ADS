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

    double OR_LATERAL_VELOCITY = 73.969; //[ft/s]
    double OR_VERTICAL_VELOCITY = 721.373; //[ft/s]
    double OR_PITCH_RATE = -1.73E-04; // [r/s]
    double OR_YAW_RATE = -5.27E-06; // [r/s]
    double OR_AZIMUTH = 0.017; // [deg]
    double OR_ZENITH = 84.166; // [deg]
    double initial_h = 964.927; // [ft]

    // From OpenRocket
    Eigen::Vector3d initial_v_world(0, -OR_LATERAL_VELOCITY, OR_VERTICAL_VELOCITY); // good
    Eigen::Vector3d initial_omega(0, OR_PITCH_RATE*2*M_PI, OR_YAW_RATE*2*M_PI); // good
    Eigen::Vector3d initial_theta(0, OR_AZIMUTH*M_PI/180.0, (90-OR_ZENITH)*M_PI/180.0); // not sure

    // For Testing
    //Eigen::Vector3d initial_theta(0, 0, 0);
    //Eigen::Vector3d initial_omega(0, 0, 0);
    //Eigen::Vector3d initial_v_body(700, 0, 0); //temp

    //convert from WORLD to BODY using R_BW (R_WB inverse)
    Eigen::Vector3d initial_v_body = getRinv(q(Eigen::Vector3d(0,0,0), initial_omega, initial_theta, initial_h, 0))*initial_v_world; 


    // post boost initial state
    q currentState(initial_v_body, initial_omega, initial_theta, initial_h, 0);

    std::cout << currentState << std::endl;
    std::cout << getApogee_testing(currentState)<< std::endl;

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