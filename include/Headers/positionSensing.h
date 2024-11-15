#ifndef POSITION_SENSING_H
#define POSITION_SENSING_H

#include <vector>
#include <cmath>


class positionSensing {
    public:
        // Current position in Cartesian coordinates
        std::vector<float> currentPosition;

        // Distances for tracking wheels
        float leftTrackingWheelDistance;
        float rightTrackingWheelDistance;
        float backTrackingWheelDistance;

        // Last reset orientation
        float lastResetGlobalOrientation;
    public:
        // Constructor
        positionSensing(float startingX, float startingY, float _leftTrackingWheelDistance, float _rightTrackingWheelDistance, float _backTrackingWheelDistance, float startingOrientation);

        // Get current position
        std::vector<float> GetPosition();

        // Update position based on wheel angles
        void UpdatePosition(float angleLeft, float angleRight, float angleBack);


};


#endif // POSITION_SENSING_H
