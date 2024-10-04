#include <vector>
#ifndef HEADER_HPP
#define HEADER_HPP
class PositionSensing{
    public: PositionSensing(float startingX, float startingY, float leftTrackingWheelDistance, float rightTrackingWheelDistance, float backTrackingWheelDistance, float startingOrientation);
    public: std::vector<float> GetPosition();
    public: void UpdatePosition(float deltaLeft, float deltaRight, float deltaBack);
};

class VectorMath {
    public: static void CartesianToPolar(std::vector<float> &componentVector);
    public: static void PolarToCartesian(std::vector<float> &polarVector);
    public: static float AngleToDistance(float angle);
    public: static float RadiansToDegrees(float radians);
    public: static float DegreesToRadians(float degrees);
    public: static void ScaleVector(std::vector<float> &cartesianVector, float scalar);
    public: static void AdditionVector(std::vector<float> &Vector, float constant);
    public: static void RotateVectorAddition(std::vector<float> &polarVector, float radians);
    public: static void AddVectors(std::vector<float> &holderVector, std::vector<float> &addedVector);
    public: static float Clampf(float value, float minimum, float maximum);
};
#endif
