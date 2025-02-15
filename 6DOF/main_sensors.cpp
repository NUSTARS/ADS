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
#include <cstdlib> 


//before this you need to 

int main_loop_dt(double vx, double vy, double vz, double wx, double wy, double wz, double theta_x, double theta_y, double theta_z, double initial_h, double u) {



    // From OpenRocket
    Eigen::Vector3d initial_v_body(vx,vy,vz); 
    Eigen::Vector3d initial_omega(wx,wy,wz);  
    Eigen::Vector3d initial_theta(theta_x, theta_y, theta_z);  

    // post boost initial state
    q currentState(initial_v_body, initial_omega, initial_theta, initial_h, u);

    Wind wind;

    //call SHISHIR CODE to get b, set B in current state going forward
    double signal = find_u(currentState, 5000, 10); //forward integrate is placeholder
    
    std::cout << current_t << std::endl;

    currentState.setU(signal);
    // std::cout << currentState << std::endl;
    currentState = integrate(currentState, &wind);
    // std::cout << currentState << std::endl;
    current_t += DT;

    }
};