// #include <iostream>
#include <ArduinoEigen.h>
#include "q.h"
#include "constants.h"
#include "simulator.h"
#include <random>
#include "sensing.h"

Eigen::Vector3d addGaussianNoise(Eigen::Vector3d value, double mean, double stddev) {
    static std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);

    Eigen::Vector3d noiseVector;
    noiseVector[0] = value[0] + distribution(generator);
    noiseVector[1] = value[1] + distribution(generator);
    noiseVector[2] = value[2] + distribution(generator);

    return noiseVector;
}

double addGaussianNoise(double value, double mean, double stddev) {
    static std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);
    return value + distribution(generator);
}


q addGaussianNoise(q currentState, double IMUmean, double IMUstddev, double BMPmean, double BMPstddev) {
    return q(addGaussianNoise(currentState.getV(), IMUmean, IMUstddev),  
                        addGaussianNoise(currentState.getOmega(), IMUmean, IMUstddev),
                        addGaussianNoise(currentState.getTheta(), IMUmean, IMUstddev), 
                        addGaussianNoise(currentState.getH(), BMPmean, BMPstddev),
                        addGaussianNoise(currentState.getU(), BMPmean, BMPstddev));
}

Eigen::Vector3d calcVelocity(q curr_q, double IMUmean, double IMUstddev, double BMPmean, double BMPstddev, Wind* wind) {
    //cannot apply error directly to velocity because never measure it 
    
    q k1 = addGaussianNoise(getqdot(curr_q, wind), IMUmean, IMUstddev,BMPmean, BMPstddev) * DT;
    q k2 = addGaussianNoise(getqdot(curr_q + k1/2.0, wind), IMUmean, IMUstddev,BMPmean, BMPstddev) * DT;
    q k3 = addGaussianNoise(getqdot(curr_q + k2/2.0, wind), IMUmean, IMUstddev,BMPmean, BMPstddev) * DT;
    q k4 = addGaussianNoise(getqdot(curr_q + k3, wind), IMUmean, IMUstddev,BMPmean, BMPstddev) * DT;
    q new_q = curr_q + (k1 + k2*2.0 + k3*2.0 + k4) * (1/6.0);
    Eigen::Vector3d new_velocity = new_q.getV();
    return new_velocity; 
    
}



q addSensorNoise(q currentState, Wind* wind) {

    //add noise to the IMU and barometer (velocity)

    Eigen::Vector3d new_vel = calcVelocity(currentState, IMU_NOISE_MEAN, IMU_NOISE_STDDEV, BMP_NOISE_MEAN, BMP_NOISE_STDDEV, wind);
    
    return q(new_vel, 
            addGaussianNoise(currentState.getOmega(),IMU_NOISE_MEAN, IMU_NOISE_STDDEV), 
            addGaussianNoise(currentState.getTheta(), IMU_NOISE_MEAN, IMU_NOISE_STDDEV), 
            addGaussianNoise(currentState.getH(), BMP_NOISE_MEAN, BMP_NOISE_STDDEV), 
            currentState.getU());
}