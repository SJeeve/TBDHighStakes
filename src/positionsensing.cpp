#include "vex.h"
#include "vector"
#include <vex.h>

using namespace vex;

//Note: All measurements are in inches

    positionSensing::positionSensing(float startingX, float startingY, float _leftTrackingWheelDistance, float _rightTrackingWheelDistance, float _backTrackingWheelDistance, float startingOrientation)
    {
        currentPosition = {startingX, startingY};
        leftTrackingWheelDistance = _leftTrackingWheelDistance;
        rightTrackingWheelDistance = _rightTrackingWheelDistance;
        backTrackingWheelDistance = _backTrackingWheelDistance;
        lastResetGlobalOrientation = startingOrientation;
    }

    std::vector<float> positionSensing::GetPosition()
    {
        return currentPosition;
    }

    void positionSensing::UpdatePosition(float angleLeft, float angleRight, float angleBack)
    {
        //Deltas correspond to the change in angles from the rotation sensors
        //Need to change deltas to distance traveled 
        float deltaLeft = vectorMath::AngleToDistance(angleLeft);
        float deltaRight = vectorMath::AngleToDistance(angleRight);
        float deltaBack = vectorMath::AngleToDistance(angleBack);
        //Note arc angle is in radians
        float deltaTheta = (deltaLeft - deltaRight) / (leftTrackingWheelDistance + rightTrackingWheelDistance);
        float arcRadius = (deltaRight / deltaTheta) + rightTrackingWheelDistance;

        std::vector<float> globalTranslationVector = {(deltaBack / deltaTheta) + backTrackingWheelDistance, (deltaRight / deltaTheta) + rightTrackingWheelDistance};
        //Have to multiply vector by 2sin(theta/2) t
        globalTranslationVector = vectorMath::ScaleVector(globalTranslationVector, 2 * sin(deltaTheta / 2.0));
        vectorMath::CartesianToPolar(globalTranslationVector);
        vectorMath::RotateVectorAddition(globalTranslationVector, (lastResetGlobalOrientation + deltaTheta / 2.0));
        vectorMath::PolarToCartesian(globalTranslationVector);
        currentPosition = vectorMath::AddVectors(currentPosition, globalTranslationVector);
    }
