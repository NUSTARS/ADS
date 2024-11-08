#include <iostream>
#include <Eigen/Core>
#include "q.h"
// #include "calcDynamics.h"
// #include "constants.h"
#include <random>


double addGaussianNoise(double value, double mean, double stddev) {
    static std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);
    return value + distribution(generator);
}

q addSensorNoise(const q& currentState) {
    double noise_mean = 0.0;
    double noise_stddev = 0.01; // Standard deviation of noise, adjust as needed

   /*return q(
        addGaussianNoise(currentState.getVX(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getVY(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getVZ(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getOmegaX(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getOmegaY(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getOmegaZ(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getTheta(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getPsi(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getPhi(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getH(), noise_mean, noise_stddev),
        addGaussianNoise(currentState.getU(), noise_mean, noise_stddev)
    );*/
    return q(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0), 0, 0);
}

int main() {
    // post boost initial state -- fill in with real numbers
    q currentState(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), 0, 0);
    double dt = 0.1;
    double max_simTime = 30;
    double current_t = 0.0;

    //run while the sim is less than 30 seconds and havent reached apogee 
    // while ((current_t < max_simTime) && !(at_apogee(currentState))) {

    //     q sensorReadingState = addSensorNoise(currentState);
    //     currentState = simulate(sensorReadingState, dt);

    // }



    // Define a 2x2 matrix with predefined values
    Eigen::Matrix2d matA;
    matA << 1, 2,
            3, 4;

    // Define another 2x2 matrix
    Eigen::Matrix2d matB;
    matB << 5, 6, 
            7, 8;

    // Print the matrices
    std::cout << "Matrix A:\n" << matA << "\n\n";
    std::cout << "Matrix B:\n" << matB << "\n\n";

    // Matrix addition
    Eigen::Matrix2d matSum = matA + matB;
    std::cout << "A + B:\n" << matSum << "\n\n";

};