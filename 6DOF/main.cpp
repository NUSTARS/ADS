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

    // From OpenRocket
    Eigen::Vector3d initial_v_world(0, 74.223, 725.333); // they only give it in world
    Eigen::Vector3d initial_omega(0,	-3.52E-04,	3.74E-04); // good
    Eigen::Vector3d initial_theta(0, 0.0182*M_PI/180,84.158 * M_PI/180); // should check these (roll, ?,?) --> radians
    double initial_h = 928.76; // good

    // For Testing
    // Eigen::Vector3d initial_v_world(0, 200, 694.52);
    // Eigen::Vector3d initial_theta(0, 0, 0);
    // Eigen::Vector3d initial_omega(0, 0, 0);

    //convert from WORLD to BODY using R_BW (R_WB inverse)
    Eigen::Vector3d initial_v_body = getR(q(Eigen::Vector3d(0,0,0), initial_omega, initial_theta, initial_h, 0)).inverse()*initial_v_world; 
    // Eigen::Vector3d initial_v_body(700, 100, 0); //temp

    std::cout << "inital v body: " << initial_v_body << std::endl;

    // post boost initial state
    q currentState(initial_v_body, initial_omega, initial_theta, initial_h, 0);
    q q2 = currentState;

    std::cout << q2 << std::endl;
    getApogee(currentState);

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