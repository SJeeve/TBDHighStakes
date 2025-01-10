
#include "vex.h"
#include "vector"

using namespace vex;
//Note: All measurements are in inches
    
    positionSensing::positionSensing(double x, double y, double theta)
    {
        currentPosition = new std::vector<double>();
        startingX = x;
        startingY = y;
        currentPosition->push_back(startingX);
        currentPosition->push_back(startingY);
        lastResetGlobalOrientation = theta;
        thetaSub0 = lastResetGlobalOrientation;
    }

    /// @brief Returns the current position of the robot
    /// @return 
    std::vector<double>* positionSensing::GetPosition()
    {
        return currentPosition;
    }

    void positionSensing::resetRotationSensors()
    {
        leftTrackingWheel.resetPosition();
        rightTrackingWheel.resetPosition();
        backTrackingWheel.resetPosition();
    }
    /// @brief Updates position with current angles in rotation sensors
    /// @param angleLeft 
    /// @param angleRight 
    /// @param angleBack 
    void positionSensing::UpdatePosition()
    {
        //DEltas correspond to the change in angles from the rotation sensors
        //need to change deltas to distance traveled 
        double deltaLeft = vectorMath::AngleToDistance(leftTrackingWheel.position(degrees));
        double deltaRight = vectorMath::AngleToDistance(rightTrackingWheel.position(degrees));
        double deltaBack = vectorMath::AngleToDistance(backTrackingWheel.position(degrees));
        Brain.Screen.printAt(0,110, "deltaLeft: %f",deltaLeft );
        Brain.Screen.printAt(0,130, "deltaRight: %f",deltaRight);
        Brain.Screen.printAt(0,150, "deltaBack: %f",deltaBack);
        //Note arc angle is in degrees
        double newOrientation = thetaSub0 + (deltaLeft - deltaRight) / (leftTrackingWheelDistance + rightTrackingWheelDistance);

        double deltaTheta = newOrientation - thetaSub0;


        std::vector<double>* localTranslationVector = new std::vector<double>();

        if(deltaTheta != 0)
        {
            double arcRadius = (deltaRight / deltaTheta) + rightTrackingWheelDistance;
            localTranslationVector -> push_back((deltaBack / deltaTheta) + backTrackingWheelDistance);
            localTranslationVector -> push_back(arcRadius);
            //Have to multiply vector by 2sin(theta/2) t
            vectorMath::ScaleVector(*localTranslationVector, 2 * sin(deltaTheta / 2.0));
        } else {
            localTranslationVector -> push_back(deltaBack);
            localTranslationVector -> push_back(deltaRight);
        }

        double averageOrientation = thetaSub0 + deltaTheta / 2;

        vectorMath::CartesianToPolar(*localTranslationVector);

        vectorMath::RotateVectorAddition(*localTranslationVector, -1* (averageOrientation));

        vectorMath::PolarToCartesian(*localTranslationVector);

        currentPosition -> at(0) += localTranslationVector -> at(0);
        currentPosition -> at(1) += localTranslationVector -> at(1);

        Brain.Screen.printAt(0, 190, "TranslationX: %f", localTranslationVector -> at(0));
        Brain.Screen.printAt(250, 190, "TranslationY: %f", localTranslationVector -> at(1));
        Brain.Screen.printAt(0, 210, "currentPosX: %f", currentPosition -> at(0));
        Brain.Screen.printAt(250, 210, "currentPosY: %f", currentPosition -> at(1));
        //Brain.Screen.printAt(250, 170, "%d", (currentPosition[0] == currentPosition[0]));
        thetaSub0 = newOrientation;
        resetRotationSensors();
        delete localTranslationVector;
    }
