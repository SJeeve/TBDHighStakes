#include <vector>
#ifndef HEADER_HPP
#define HEADER_HPP
class PositionSensing{
    public: PositionSensing(long startingX, long startingY, long leftTrackingWheelDistance, long rightTrackingWheelDistance, long backTrackingWheelDistance);
    public: std::vector<long> GetPosition();
    public: void UpdatePosition(long deltaLeft, long deltaRight, long deltaBack);
};

class VectorMath {
    public: static std::vector<long> CartesianToPolar(std::vector<long> componentVector);
    public: static std::vector<long> PolarToCartesian(std::vector<long> polarVector);
    public: static double AngleToDistance(double angle);
    public: static double RadiansToDegrees(double radians);
    public: static double DegreesToRadians(double degrees);
    public: static void ScaleVector(std::vector<double> &cartesianVector, double scalar);
};
#endif