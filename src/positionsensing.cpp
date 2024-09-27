#include "vex.h"
#include "vector"
#include "Header.hpp"
using namespace vex;
    class PositionSensing{
        public:
        std::vector<long> previousGlobalPosition;
        long previousGlobalOrientation;
        long lastResetGlobalOrientation;

        //Left-right distance between tracking center and left tracking wheel
        long leftTrackingWheelDistance;
        //Left-right distance between traclong center and right tracking wheel
        long rightTrackingWheelDistance;
        //Forward-backward distance between tracking center to the back tracking wheel
        long backTrackingWheelDistance;
        PositionSensing(long startingX, long startingY, long _leftTrackingWheelDistance, long _rightTrackingWheelDistance, long _backTrackingWheelDistance)
        {
            previousGlobalPosition = {startingX, startingY};
            leftTrackingWheelDistance = _leftTrackingWheelDistance;
            rightTrackingWheelDistance = _rightTrackingWheelDistance;
            backTrackingWheelDistance = _backTrackingWheelDistance;
        }

        public: std::vector<long> GetPosition()
        {
            return previousGlobalPosition;
        }

        public: void UpdatePosition(long deltaLeft, long deltaRight, long deltaBack)
        {
            //Deltas correspond to the change in angles from the rotation sensors
            
        }
};