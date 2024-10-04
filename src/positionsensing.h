#ifndef POSITION_SENSING_H
#define POSITION_SENSING_H

#include <vector>
#include <cmath>

namespace positionsensing {

class PositionSensing {
public:
    // Constructor
    PositionSensing(float, float,
                    float,
                    float,
                    float,
                    float);

    // Get current position
    std::vector<float> GetPosition();

    // Update position based on wheel angles
    void UpdatePosition(float angleLeft, float angleRight, float angleBack);

private:
    // Current position in Cartesian coordinates
    std::vector<float> currentPosition;

    // Distances for tracking wheels
    float leftTrackingWheelDistance;
    float rightTrackingWheelDistance;
    float backTrackingWheelDistance;

    // Last reset orientation
    float lastResetGlobalOrientation;
};

} // namespace positionsensing

#endif // POSITION_SENSING_H
