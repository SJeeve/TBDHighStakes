#ifndef POSITION_SENSING_H
#define POSITION_SENSING_H

#include "vex.h"
#include <vector>
#include <cmath>
#include <typeinfo>

class positionSensing {
    public:
        // Current position in Cartesian coordinates
        
        // Distances for tracking wheels
        //Last orientation
        double thetaSub0;
        //Initial orientation
        double lastResetGlobalOrientation;
        double startingX;
        double startingY;

    public:
        // Constructor
        positionSensing(double x, double y, double theta);
        void resetRotationSensors();
        // Get current position
        std::vector<double>* GetPosition();

        // Update position based on wheel angles
        void UpdatePosition();


};


#endif // POSITION_SENSING_H
