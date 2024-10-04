#include "vex.h"
#include "vector"
#include "positionsensing.h"
#include "vectorMath.cpp"

using namespace vex;
using namespace vectormath;

//Note: All measurements are in inches
namespace positionsensing{

    positionsensing::PositionSensing(float startingX, float startingY, float _leftTrackingWheelDistance, float _rightTrackingWheelDistance, float _backTrackingWheelDistance, float startingOrientation)
    {
        currentPosition = {startingX, startingY};
        leftTrackingWheelDistance = _leftTrackingWheelDistance;
        rightTrackingWheelDistance = _rightTrackingWheelDistance;
        backTrackingWheelDistance = _backTrackingWheelDistance;
        lastResetGlobalOrientation = startingOrientation;
    }

    std::vector<float> positionsensing::GetPosition()
    {
        return currentPosition;
    }

    void positionsensing::UpdatePosition(float angleLeft, float angleRight, float angleBack)
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
        vectormath::VectorMath::ScaleVector(globalTranslationVector, 2 * sin(deltaTheta / 2.0));
        vectormath::VectorMath::CartesianToPolar(globalTranslationVector);
        vectormath::VectorMath::RotateVectorAddition(globalTranslationVector, (lastResetGlobalOrientation + deltaTheta / 2.0));
        vectormath::VectorMath::PolarToCartesian(globalTranslationVector);
        vectormath::VectorMath::AddVectors(currentPosition, globalTranslationVector);
    }
}