#include "controls.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <functional>
#include "simulator.h"
#include <Eigen/Core>
#include "q.h"
#include "constants.h"

double findU(q states, double apogee, double err) {

    double b = 0.0;

    // std::cout << getApogee(states) << std::endl;
    // std::cout << getApogee(states, -RAMP_TIME) << std::endl;
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

        while(abs(currError) > err){
            if(currError > 0){ // current apogee too high --> actuate earlier
                max = avg;
            }
            else{ // current apogee too low --> actuate later
                min = avg;
            }
            avg = (max+min)/2.0;
            currError = getApogee(states, avg) - apogee;
        }

        b = avg;


    }

    //std::cout << getApogee(states, b) << std::endl;

    return F(-b);

}