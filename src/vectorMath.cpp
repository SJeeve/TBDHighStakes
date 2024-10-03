#include <vex.h>
#include <vector>
#include <iostream>

using namespace vex;
using namespace std;

    class VectorMath{
        public:
        //Wheel diameter is somewhere between 2.5 and 2.75
        //Needs more testing to get actual value
        const static float wheelDiameter = 2.75;
        const static float wheelCircumference = wheelDiameter * M_PI;
        public: static void CartesianToPolar(std::vector<long> &componentVector)
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
        public: static void AddVectors(std::vector<float> &holderVector, std::vector<float> &addedVector)
        {
            //This adds one vector to another
            //Note that the vector designated holderVector is the only vector that will be changed
            //Although not required, this should only be used with cartesian vectors
            for(int i = 0; i < holderVector.size(); i++)
                holderVector[i] += addedVector[i];
        }
        public: static void RotateVectorAddition(std::vector<float> &polarVector, float radians)
        {
            //This requires the vector to be in polar form
            polarVector[1] += radians;
        }

        public: static void PolarToCartesian(std::vector<float> &polarVector)
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

        public: static void ScaleVector(std::vector<float> &cartesianVector, float scalar)
        {
            for(int i = 0; i < cartesianVector.size(); i++)
                cartesianVector[i] *= scalar;
        }    

        public: static float AngleToDistance(float angle)
        {
            return (angle / 360.0) * wheelCircumference;
        }

        public: static float RadiansToDegrees(float radians)
        {
            return radians * 180.0 / M_PI;
        }

        public: static float DegreesToRadians(float degrees)
        {
            return degrees * M_PI / 180.0;
        }
    };