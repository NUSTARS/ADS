#include "wind.h"
#include "constants.h"
#include <iostream>

/*wind.cpp*/

//
// Class for storing and computing wind data
//
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025
//


//
// calcWind
// 
// Calculates Wind for a single timestep
//
void Wind::calcWind(){
    double w =  dist(generator); // gaussian white noise

    //calculate noise using pink noise formula, normalizing by wind std
    double pinkNoise = w - a_values[1]*windSpeed[1] - a_values[2]*windDirection[1];
    double windSpeedVal= WIND_VELOCITY + WIND_STD*(pinkNoise/2.252);

    // do the same for dir
    w = dist(generator);
    pinkNoise = w - a_values[1]*windSpeed[1] - a_values[2]*windDirection[1];
    double windDirVal = WIND_ANGLE + WIND_STD*(pinkNoise/2.252);
    //double windDirVal = WIND_ANGLE; // for testing

    // std::cout << windSpeedVal << std::endl;


    updateWindArrays(windSpeedVal, windDirVal);

}

//
// updateWindArrays
//
// Cascades wind arrays so old values are updated accordingly
//
void Wind::updateWindArrays(double newWindSpeed, double newWindDirection){

    windSpeed[2] = windSpeed[1];
    windSpeed[1] = windSpeed[0];

    windDirection[2] = windDirection[1];
    windDirection[1] = windDirection[0];
  
    windSpeed[0] = newWindSpeed;
    windDirection[0] = newWindDirection;

    // std::cout << windSpeed[0] << std::endl;

}

//
// generateA
//
// generates array of a values based on OR formula
//
double* Wind::generateA(int numValues){

    double* a_values = new double[numValues];
    a_values[0] = 0;

    for(int k = 1; k < numValues; k++){
        a_values[k] = (k - 1.0 - ALPHA/2.0) * (a_values[k-1]/k);
    }

    return a_values;


}

//
// constructor
//
Wind::Wind(){
    std::random_device rd;  // Non-deterministic random seed
    generator.seed(rd());
    dist = std::normal_distribution<double>(0,1);
    a_values = generateA(3);
    windDirection[0] = 0;
    windDirection[1] = 0;
    windDirection[2] = 0;
    windSpeed[0] = 0;
    windSpeed[1] = 0;
    windSpeed[2] = 0;



}

//
// updateWind
//
// GeneratesWind for next sample
//
void Wind::updateWind(){
    calcWind();
}
   
//
// getWind
//
// Gets the current wind as a 2D vector
//
Eigen::Vector2d Wind::getWind(){
    Eigen::Vector2d dir(sin(windDirection[0]* M_PI / 180.0),cos(windDirection[0]* M_PI / 180.0));
    // std::cout << windSpeed[0]*dir << std::endl;
    return windSpeed[0]*dir;
}