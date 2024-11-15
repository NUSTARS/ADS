#ifndef SENSING_H
#define SENSING_H

#include <iostream>
#include <Eigen/Core>
#include "q.h"


Eigen::Vector3d addGaussianNoise(Eigen::Vector3d value, double mean, double stddev);


double addGaussianNoise(double value, double mean, double stddev);

q addGaussianNoise(q currentState, double IMUmean, double IMUstddev, double BMPmean, double BMPstddev);

Eigen::Vector3d calcVelocity(q curr_q, double IMUmean, double IMUstddev, double BMPmean, double BMPstddev);

q addSensorNoise(q currentState);

#endif

