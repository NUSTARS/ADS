#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "main_sensors.h"
#include "q.h"
#include "simulator.h"


int main() {
    std::ifstream file("vdf2.csv");
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    // Skip header line
    // std::getline(file, line);

    int count = 0;
    while (std::getline(file, line) && count < 1000) {
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> fields;

        while (std::getline(ss, value, ',')) {
            fields.push_back(value);
        }

        if (fields.size() < 18) {
            std::cerr << "Row does not have enough columns!" << std::endl;
            continue;
        }

        // Extract relevant columns
        double theta_x = std::stod(fields[3]); // Euler X [rad]
        double theta_y = std::stod(fields[5]); // Euler Y [rad]
        double theta_z = std::stod(fields[7]); // Euler Z [rad]

        double wx = std::stod(fields[8]); // Omega X
        double wy = std::stod(fields[9]); // Omega Y
        double wz = std::stod(fields[10]); // Omega Z

        double vx = std::stod(fields[14]); // Vel X
        double vy = std::stod(fields[15]); // Vel Y
        double vz = std::stod(fields[16]); // Vel Z

        double height = std::stod(fields[1]); // Altitude [ft]
        double u = std::stod(fields[17]); // U

        Eigen::Vector3d initial_v(vx, vy, vz); 
        Eigen::Vector3d initial_omega(wx, wy, wz);
        Eigen::Vector3d initial_theta(theta_x, theta_y, theta_z); 

        u = main_loop_dt(vz, vy, vx, wz, wy, wx, theta_x, theta_y, theta_z, height, u);

        q currentState(initial_v, initial_omega, initial_theta, height, u);

        std::cout << "altitude: " << height << " predicted apogee " << getApogee(currentState) << "  actuated apogee: " << getApogee(currentState, 0) << " control signal: " << u;
        count += 1;
    }

    file.close();
    return 0;
}