#include <iostream>
#include <Eigen/Core>
#include "q.h"
#include "calcDynamics.h"
#include "constants.h"
#include "simulator.h"
#include <random>
#include "controls.h"
#include "sensing.h"

int main() {

    Eigen::Vector3d initial_v_world(0, 204.934, 694.52); //ft/s

    Eigen::Vector2d* old_wind = new Eigen::Vector2d(0,0);

    // post boost initial state -- fill in with real numbers
    q currentState(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), 0, 0);

    double max_simTime = 30;
    double current_t = 0.0;


    // run while the sim is less than 30 seconds and havent reached apogee 
    while ((current_t < max_simTime) && !(atApogee(currentState))) {

        q SensorNoiseState = addSensorNoise(currentState);

        //call SHISHIR CODE to get b, set B in current state going forward
        double signal = find_u(SensorNoiseState, 5150, 10); //forward integrate is placeholder
        std::cout<< signal << std::endl;

        SensorNoiseState.setU(signal);
        q currentState = integrate(SensorNoiseState, old_wind);

        current_t += DT;

    }


};