/*q.h*/

//
// Wrapper class storing the current state (or derivative of the state) of the rocket
//
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025
//
#ifndef Q_H
#define Q_H

#include <Eigen/Core>


class q {
    private:
        //
        // State variables for state equations
        //

        Eigen::Vector3d v; // Linear velocities
        Eigen::Vector3d omega; // Angular velocities
        Eigen::Vector3d theta; // Euler angle rotation
        double h; // Altitude

        double u; // Current control signal for flaps: 0 = fully closed, 1 = fully opened


    public:
        //
        // constructor (maybe make it take in an array instead?)
        //
        q(Eigen::Vector3d newV, Eigen::Vector3d newOmega, Eigen::Vector3d newTheta, double newH, double newU);
        
        //
        // copy constructor:
        //
        q(const q& other);

        //
        // operators
        //
        q operator+(const q& obj);
        q operator*(double a);
        q operator/(double a);
        bool operator == (const q &q);


        //
        // Accessors
        //
        Eigen::Vector3d getV() const;
        Eigen::Vector3d getOmega() const;
        Eigen::Vector3d getTheta() const;      
        double getH() const;

        double getU() const;

        //
        // Mutator (only for u since the rest of the state cannot be adusted)
        //
        void setU(double newU);


};

q operator*(double a, const q& obj);
std::ostream & operator << (std::ostream &out, const q &q);

#endif // Q_H