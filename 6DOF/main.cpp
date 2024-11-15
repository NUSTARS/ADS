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

int main() {

    // From OpenRocket
    Eigen::Vector3d initial_v_world(0, 204.934, 694.52); // they only give it in world
    Eigen::Vector3d initial_omega(7.62E-09, 0.014, -1.19E-05); // good
    Eigen::Vector3d initial_theta(0.009*0.01745, 72.505*0.01745, 0); // should check these
    double initial_h = 898.599; // good

    //convert from WORLD to BODY using R_BW (R_WB inverse)
    Eigen::Vector3d initial_v_body = getR(q(Eigen::Vector3d(0,0,0), initial_omega, initial_theta, initial_h, 0)).inverse()*initial_v_world; 

    // post boost initial state
    q currentState(initial_v_body, initial_omega, initial_theta, initial_h, 0);

    std::cout << currentState << std::endl;
    std::cout << "Apogee Reached: " << getApogee(currentState) << std::endl;

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