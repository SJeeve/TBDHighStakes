#include <vector>
#ifndef HEADER_HPP
#define HEADER_HPP
class PositionSensing{
    public: PositionSensing(double startingX, double startingY, double leftTrackingWheelDistance, double rightTrackingWheelDistance, double backTrackingWheelDistance, double startingOrientation);
    public: std::vector<double> GetPosition();
    public: void UpdatePosition(double deltaLeft, double deltaRight, double deltaBack);
};

class VectorMath {
    public: static void CartesianToPolar(std::vector<double> componentVector);
    public: static void PolarToCartesian(std::vector<double> polarVector);
    public: static double AngleToDistance(double angle);
    public: static double RadiansToDegrees(double radians);
    public: static double DegreesToRadians(double degrees);
    public: static void ScaleVector(std::vector<double> &cartesianVector, double scalar);
    public: static void AdditionVector(std::vector<double> &Vector, double constant);
    public: static void RotateVectorAddition(std::vector<double> &polarVector, double radians);
    public: static void AddVectors(std::vector<double> &holderVector, std::vector<double> &addedVector);
};
#endif