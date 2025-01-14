#include "wind.h"
#include "constants.h"

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
    double pinkNoise = w - a_values[1]*windSpeed[1] - a_values[2]*windDirection[1];
    double windDirVal= WIND_ANGLE + WIND_STD*(pinkNoise/2.252);

    updateWindArrays(windSpeedVal, windDirVal);

}

//
// updateWindArrays
//
// Cascades wind arrays so old values are updated accordingly
//
void Wind::updateWindArrays(double newWindSpeed, double newWindDirection){
    for(int i = sizeof(windSpeed)/sizeof(double)-1; i > 0; i--){
        windSpeed[i+1] = windSpeed[i];
        windDirection[i+1] = windDirection[i];
    }
    windSpeed[0] = newWindSpeed;
    windDirection[0] = newWindDirection;
}

//
// generateA
//
// generates array of a values based on OR formula
//
double* generateA(int numValues){

    double a_values[numValues];
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
    dist = std::normal_distribution<double>(0,1);
    a_values = generateA(3);
}

//
// getWind
//
// Gets the current wind as a 2D vector
//
Eigen::Vector2d Wind::getWind(){
    calcWind();
    Eigen::Vector2d dir(sin(windDirection[0]),cos(windDirection[1]));
    return windSpeed[0]*dir;
}