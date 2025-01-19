#include "controls.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <functional>
#include "simulator.h"
#include <Eigen/Core>
#include "q.h"
#include "constants.h"

std::vector<double> createRange(double start, double end, double step) {
    std::vector<double> range;
    for (double value = start; value <= end; value += step) {
        range.push_back(value);
    }
    return range;
}

double binary_search(const std::vector<double>& b, 
                            double apogee, 
                            double err,
                            q states) {
    int left = 0;
    int right = b.size() - 1;

    while (left <= right) {
        int mid = left + (right - left) / 2;
        double bMid = b[mid];
        double calcApogee = getApogee(states, bMid); // forward project to find predicted apogee
        //std::cout << bMid << std::endl;

        // Check if calculate apogee is within error margin
        if (std::fabs(calcApogee - apogee) <= err) {
            return bMid; 
        }

        // Adjust the search range based on the calculated value
        if (calcApogee < apogee) {
            left = mid + 1;  // Use the larger half
        } 
        else {
            right = mid - 1; // Use the smaller half
        }
    }

    throw std::runtime_error("We done fucked up");
}

std::vector<double> generateRampingFunction(double duration, double b, double f, double timestep) {
    std::vector<double> rampingValues;
    double currentTime = 0.0;

    while (currentTime <= duration) {
        if (currentTime < b) {
            // Before ramping starts, the value is 0
            rampingValues.push_back(0.0);
        } else if (currentTime >= b && currentTime <= b + f) {
            // During the ramping phase, linearly increase from 0 to 1 (can change to some other behavior)
            double rampValue = (currentTime - b) / f;
            rampingValues.push_back(rampValue);
        } else {
            // After ramping is complete, the value remains at 1
            rampingValues.push_back(1.0);
        }

        currentTime += timestep;
    }

    return rampingValues;
}

double find_u(q states, double apogee, double err) {
	double f = 0.5;	  
	double timestep = DT;
	std::vector<double> b_list = createRange(-100, 100, timestep);
	double b_final = binary_search(b_list, apogee, err, states);
	if (b_final < -1*f) { // if b is less than ramp time, prolly wanna get those flaps out
		return 1.0;
	}	
	
	std::vector<double> ramp = generateRampingFunction(20.0, b_final, f, timestep);
	return ramp[0]; 

}