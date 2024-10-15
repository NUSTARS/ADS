/*q.h*/

//
// Wrapper class storing the current state (or derivative of the state) of the rocket
//
// ADS 6DOF Simulator
// Shishir Bandapalli, Max Hughes, Tara Saxena, Preston Shin
// NUSTARS NSL 2025
//

class q{
    private:
        //
        // State variables for state equations
        //

        double vX; // Linear velocities
        double vY;
        double vZ;

        double omegaX; // Angular velocities
        double omegaY;
        double omegaZ;

        double theta; // Euler angle rotation
        double psi;
        double phi;

        double h; // Altitude

        double u; // Current control signal for flaps: 0 = fully closed, 1 = fully opened


    public:
        //
        // constructor (maybe make it take in an array instead?)
        //
        q(double newVX, double newVY, double newVZ, double newOmegaX, double newOmegaY, double newOmegaZ, double newTheta, double newPsi, double newPhi, double newH, double newU);
        
        //
        // copy constructor:
        //
        q(const q& other);


        //
        // Accessors
        //

        double getVX() const;
        double getVY() const;
        double getVZ() const;

        double getOmegaX() const;
        double getOmegaY() const;
        double getOmegaZ() const;

        double getTheta() const;
        double getPsi() const;
        double getPhi() const;
        
        double getH() const;

        double getU() const;

        //
        // Mutator (only for u since the rest of the state cannot be adusted)
        //
        void setU(double newU);


};