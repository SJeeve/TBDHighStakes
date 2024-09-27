#include <vector>
#ifndef HEADER_HPP
#define HEADER_HPP
class PositionSensing{
    public: PositionSensing(long startingX, long startingY, long leftTrackingWheelDistance, long rightTrackingWheelDistance, long backTrackingWheelDistance);
    public: std::vector<long> GetPosition();
    public: void UpdatePosition(long deltaLeft, long deltaRight, long deltaBack);
};

class VectorMath {
    static std::vector<long> CartesianToPolar(std::vector<long> componentVector);
    static std::vector<long> PolarToCartesian(std::vector<long> polarVector);
};
#endif