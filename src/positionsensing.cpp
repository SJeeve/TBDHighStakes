#include "vex.h"
#include "vector"
#include "Header.hpp"
using namespace vex;
class PositionSensing{
    //Note: All measurements are in inches
    public:
    std::vector<float> currentPosition;
    float previousGlobalOrientation;
    float lastResetGlobalOrientation;

    //Left-right distance between tracking center and left tracking wheel
    float leftTrackingWheelDistance;
    //Left-right distance between tracking center and right tracking wheel
    float rightTrackingWheelDistance;
    //Forward-backward distance between tracking center to the back tracking wheel
    float backTrackingWheelDistance;
    PositionSensing(float startingX, float startingY, float _leftTrackingWheelDistance, float _rightTrackingWheelDistance, float _backTrackingWheelDistance, float startingOrientation)
    {
        currentPosition = {startingX, startingY};
        leftTrackingWheelDistance = _leftTrackingWheelDistance;
        rightTrackingWheelDistance = _rightTrackingWheelDistance;
        backTrackingWheelDistance = _backTrackingWheelDistance;
        lastResetGlobalOrientation = startingOrientation;
    }

    public: std::vector<float> GetPosition()
    {
        return currentPosition;
    }

    public: void UpdatePosition(float angleLeft, float angleRight, float angleBack)
    {
        //Deltas correspond to the change in angles from the rotation sensors
        //Need to change deltas to distance traveled 
        float deltaLeft = VectorMath::AngleToDistance(angleLeft);
        float deltaRight = VectorMath::AngleToDistance(angleRight);
        float deltaBack = VectorMath::AngleToDistance(angleBack);
        //Note arc angle is in radians
        float deltaTheta = (deltaLeft - deltaRight) / (leftTrackingWheelDistance + rightTrackingWheelDistance);
        float arcRadius = (deltaRight / deltaTheta) + rightTrackingWheelDistance;

        std::vector<float> globalTranslationVector = {(deltaBack / deltaTheta) + backTrackingWheelDistance, (deltaRight / deltaTheta) + rightTrackingWheelDistance};
        //Have to multiply vector by 2sin(theta/2) t
        VectorMath::ScaleVector(globalTranslationVector, 2 * sin(deltaTheta / 2.0));
        VectorMath::CartesianToPolar(globalTranslationVector);
        VectorMath::RotateVectorAddition(globalTranslationVector, (lastResetGlobalOrientation + deltaTheta / 2.0));
        VectorMath::PolarToCartesian(globalTranslationVector);
        VectorMath::AddVectors(currentPosition, globalTranslationVector);
    }
};