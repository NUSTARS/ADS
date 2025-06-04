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

    // From Sensor
    Eigen::Vector3d initial_v_body(vz,vy,vx); // ft/s FLIPPED HERE
    Eigen::Vector3d initial_omega(wz,wy,wx); // rad/s FLIPPED HERE
    Eigen::Vector3d initial_theta(theta_x, theta_y, theta_z); // rad NO NEED TO FLIP

    // u is u units, h is altitude

    q currentState(initial_v_body, initial_omega, initial_theta, initial_h, u);

    double signal = findU(currentState, TARGET_APO, 1); 

    return signal;
};