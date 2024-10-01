#include "vex.h"
#include "vector"
#include "Header.hpp"
using namespace vex;
class PositionSensing{
    //Note: All measurements are in inches
    public:
    std::vector<double> currentPosition;
    double previousGlobalOrientation;
    double lastResetGlobalOrientation;

    //Left-right distance between tracking center and left tracking wheel
    double leftTrackingWheelDistance;
    //Left-right distance between tracking center and right tracking wheel
    double rightTrackingWheelDistance;
    //Forward-backward distance between tracking center to the back tracking wheel
    double backTrackingWheelDistance;
    PositionSensing(double startingX, double startingY, double _leftTrackingWheelDistance, double _rightTrackingWheelDistance, double _backTrackingWheelDistance, double startingOrientation)
    {
        currentPosition = {startingX, startingY};
        leftTrackingWheelDistance = _leftTrackingWheelDistance;
        rightTrackingWheelDistance = _rightTrackingWheelDistance;
        backTrackingWheelDistance = _backTrackingWheelDistance;
        lastResetGlobalOrientation = startingOrientation;
    }

    public: std::vector<double> GetPosition()
    {
        return currentPosition;
    }

    public: void UpdatePosition(double angleLeft, double angleRight, double angleBack)
    {
        //Deltas correspond to the change in angles from the rotation sensors
        //Need to change deltas to distance traveled 
        double deltaLeft = VectorMath::AngleToDistance(angleLeft);
        double deltaRight = VectorMath::AngleToDistance(angleRight);
        double deltaBack = VectorMath::AngleToDistance(angleBack);
        //Note arc angle is in radians
        double deltaTheta = (deltaLeft - deltaRight) / (leftTrackingWheelDistance + rightTrackingWheelDistance);
        double arcRadius = (deltaRight / deltaTheta) + rightTrackingWheelDistance;

        std::vector<double> globalTranslationVector = {(deltaBack / deltaTheta) + backTrackingWheelDistance, (deltaRight / deltaTheta) + rightTrackingWheelDistance};
        //Have to multiply vector by 2sin(theta/2) t
        VectorMath::ScaleVector(globalTranslationVector, 2 * sin(deltaTheta / 2.0));
        VectorMath::CartesianToPolar(globalTranslationVector);
        VectorMath::RotateVectorAddition(globalTranslationVector, (lastResetGlobalOrientation + deltaTheta / 2.0));
        VectorMath::PolarToCartesian(globalTranslationVector);
        VectorMath::

    }
};