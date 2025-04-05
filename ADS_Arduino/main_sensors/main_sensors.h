#ifndef MAINSENSORS_H
#define MAINSENSORS_H


// #include <iostream>
// #include <Eigen/Core>
#include <ArduinoEigen.h>
#include "q.h"
#include "wind.h"



double main_loop_dt(double vx, double vy, double vz, double wx, double wy, double wz, double theta_x, double theta_y, double theta_z, double initial_h, double u);



#endif 