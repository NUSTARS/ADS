/*wind.h*/

//
// Class for storing and computing wind data
//
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025
//
#ifndef WIND_H
#define WIND_H

#include <Eigen/Core>
#include <random>


class Wind {
    private:

        //
        // White gaussian noise generation 
        //
        std::default_random_engine generator;
        std::normal_distribution<double> dist;
        double* a_values;

        //
        // Past and Current Wind (1st element = current, 2nd = prev, etc)
        //
        double windSpeed[3];
        double windDirection[3];

        //
        // calcWind
        // 
        // Calculates Wind for a single timestep
        //
        void calcWind();

        //
        // updateWindArrays
        //
        // Cascades wind arrays so old values are updated accordingly
        //
        void updateWindArrays(double newWindSpeed, double newWindDirection);

        //
        // generateA
        //
        // generates array of a values based on OR formula
        //
        double* generateA(int numValues);





    public:

        //
        // constructor
        //
        Wind();

        //
        // updateWind
        //
        // GeneratesWind for next sample
        //
        void updateWind();

        //
        // getWind
        //
        // Gets the current wind as a 2D vector
        //
        Eigen::Vector2d getWind();


};

#endif // WIND_H