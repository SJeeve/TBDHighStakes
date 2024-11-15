#include <vex.h>
#include <vector>
#include <iostream>
#include <vex.h>

using namespace vex;
using namespace std;
        //Wheel diameter is somewhere between 2.5 and 2.75
        //Needs more testing to get actual value
        
        void vectorMath::CartesianToPolar(std::vector<float> &componentVector)
        {
            //NOTE: ALL THESE ARE ALL IN RADIANS NOT DEGREES
            //& means pass by reference
            float xComponent = componentVector[0];
            float yComponent = componentVector[1];
            //Sets first element of vector to length of vector
            componentVector[0] = sqrt(pow(xComponent, 2.0) + pow(yComponent, 2.0));
            //Sets second element to angle
            componentVector[1] = atan(yComponent/xComponent);
        }

        std::vector<float> vectorMath::AddVectors(std::vector<float> holderVector, std::vector<float> addedVector)
        {
            //This adds one vector to another
            //Note that the vector designated holderVector is the only vector that will be changed
            //Although not required, this should only be used with cartesian vectors
            for(int i = 0; i < holderVector.size(); i++)
                holderVector[i] += addedVector[i];
            return holderVector;
        }

        std::vector<float> vectorMath::SubtractVectors(std::vector<float> &subtractingVector, std::vector<float> subtractedVector)
        {
            for(int i = 0; i < subtractedVector.size(); i++)
                subtractedVector[i] -= subtractingVector[i];
            return subtractedVector;
        }

        static std::vector<float> NormalizeVector(std::vector<float> normalizedVector)
        {
            float lengthMultiplier = 1/vectorMath::MagnitudeOfVector(normalizedVector);
            for(int i = 0; i < normalizedVector.size(); i++)
                normalizedVector[i] *= lengthMultiplier;
            return normalizedVector;
        }

        void vectorMath::RotateVectorAddition(std::vector<float> &polarVector, float radians)
        {
            //This requires the vector to be in polar form
            polarVector[1] += radians;
        }

        float vectorMath::Clampf(float value, float minimum, float maximum)
        {
            if(value < minimum)
                return minimum;
            if(value > maximum)
                return maximum;
            return value;
        }

        float vectorMath::MagnitudeOfVector(std::vector<float> &cartesianVector)
        {
            return sqrt(pow(cartesianVector[0], 2) + pow(cartesianVector[1], 2));
        }
        
        void vectorMath::PolarToCartesian(std::vector<float> &polarVector)
        {
            //NOTE: THESE ARE ALL IN RADIANS AND NOT DEGREES
            float polarMagnitude = polarVector[0];
            float polarRadians = polarVector[1];
            polarVector[0] = polarMagnitude * cos(polarRadians);
            polarVector[1] = polarMagnitude * sin(polarRadians);
        }

        std::vector<float> vectorMath::ScaleVector(std::vector<float> cartesianVector, float scalar)
        {
            for(int i = 0; i < cartesianVector.size(); i++)
                cartesianVector[i] *= scalar;
            return cartesianVector;
        }    

        float vectorMath::AngleToDistance(float angle)
        {
            return (angle / 360.0) * wheelCircumference;
        }

        float vectorMath::RadiansToDegrees(float radians)
        {
            return radians * 180.0 / M_PI;
        }

        float vectorMath::DegreesToRadians(float degrees)
        {
            return degrees * M_PI / 180.0;
        }