#include <vex.h>
#include <vector>
#include <iostream>
#include "VectorMathHeader.h"
using namespace vex;
using namespace std;
        //Wheel diameter is somewhere between 2.5 and 2.75
        //Needs more testing to get actual value
        void VectorMath::CartesianToPolar(std::vector<float> &componentVector)
        {
            //NOTE: ALL THESE ARE ALL IN RADIANS NOT DEGREES
            //& means pass by reference
            if(componentVector.size() != 2)
            {
                std::cerr << "vectorMath CartesianToPolar warning: Vector size is " + componentVector.size() << std::endl;
            }
            float xComponent = componentVector[0];
            float yComponent = componentVector[1];
            //Sets first element of vector to length of vector
            componentVector[0] = sqrt(pow(xComponent, 2.0) + pow(yComponent, 2.0));
            //Sets second element to angle
            componentVector[1] = atan(yComponent/xComponent);
        }

        void VectorMath::AddVectors(std::vector<float> &holderVector, std::vector<float> &addedVector)
        {
            //This adds one vector to another
            //Note that the vector designated holderVector is the only vector that will be changed
            //Although not required, this should only be used with cartesian vectors
            for(int i = 0; i < holderVector.size(); i++)
                holderVector[i] += addedVector[i];
        }
        
        void VectorMath::RotateVectorAddition(std::vector<float> &polarVector, float radians)
        {
            //This requires the vector to be in polar form
            polarVector[1] += radians;
        }

        void VectorMath::PolarToCartesian(std::vector<float> &polarVector)
        {
            //NOTE: THESE ARE ALL IN RADIANS AND NOT DEGREES
            if(polarVector.size() != 2)
            {
                std::cerr << "vectorMath PolarToCartesian warning: Vector size is " + polarVector.size() << std::endl;
            }
            float polarMagnitude = polarVector[0];
            float polarRadians = polarVector[1];
            polarVector[0] = polarMagnitude * cos(polarRadians);
            polarVector[1] = polarMagnitude * sin(polarRadians);
        }

        void VectorMath::ScaleVector(std::vector<float> &cartesianVector, float scalar)
        {
            for(int i = 0; i < cartesianVector.size(); i++)
                cartesianVector[i] *= scalar;
        }    

        float VectorMath::AngleToDistance(float angle)
        {
            return (angle / 360.0) * wheelCircumference;
        }

        float VectorMath::RadiansToDegrees(float radians)
        {
            return radians * 180.0 / M_PI;
        }

        float VectorMath::DegreesToRadians(float degrees)
        {
            return degrees * M_PI / 180.0;
        }