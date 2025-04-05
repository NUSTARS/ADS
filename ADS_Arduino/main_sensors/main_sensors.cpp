// #include <iostream>
#include "main_sensors.h"
// #include <Eigen/Core>
// #include <Eigen/Dense>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include "q.h"
#include "calcDynamics.h"
#include "constants.h"
#include "simulator.h"
#include "controls.h"


 
double main_loop_dt(double vx, double vy, double vz, double wx, double wy, double wz, double theta_x, double theta_y, double theta_z, double initial_h, double u) {

    // From OpenRocket
    Eigen::Vector3d initial_v_body(vx,vy,vz); 
    Eigen::Vector3d initial_omega(wx,wy,wz);  
    Eigen::Vector3d initial_theta(theta_x, theta_y, theta_z);  

    q currentState(initial_v_body, initial_omega, initial_theta, initial_h, u);

    double signal = findU(currentState, TARGET_APO, 5); //forward integrate is placeholder

    return signal;
    // return 0.0;
};