#include "controls.h"
#include <iostream>
#include <cmath>
#include "simulator.h"
#include <Eigen/Core>
#include "q.h"
#include "constants.h"

double findU(q states, double apogee, double err) {

    double b = 0.0;

    //std:: cout << "a: " << getApogee(states, -RAMP_TIME) << " b: " << getApogee(states) << std::endl;

    if(getApogee(states, -RAMP_TIME) > apogee - err){ // check the bounds first
        b = -RAMP_TIME;
    }
    else if (getApogee(states, 0) < apogee + err){ // no flaps
        b = 0.0;
    }
    else{ // then search for non-extreme values
        double min = -RAMP_TIME;
        double max = 0;
        double avg = (min+max)/2.0;

        double currError = getApogee(states, avg) - apogee;
        int iter = 0;

        while(abs(currError) > err && iter < MAX_SEARCH_ITER){

            //std::cout << currError << ", " << avg << std::endl;


            if(currError > 0){ // current apogee too high --> actuate earlier
                max = avg;
            }
            else{ // current apogee too low --> actuate later
                min = avg;
            }
            avg = (max+min)/2.0;
            currError = getApogee(states, avg) - apogee;

            iter++;
        }

        b = avg;

        std::cout << " Error: " << currError;


    }

    std::cout << std::endl;

    return F_function(-b);
}