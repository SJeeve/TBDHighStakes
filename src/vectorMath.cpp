#include <vex.h>
#include <vector>
#include <iostream>

using namespace vex;
using namespace std;

    class VectorMath{
        public:
        //Wheel diameter is somewhere between 2.5 and 2.75
        //Needs more testing to get actual value
        const static double wheelDiameter = 2.75;
        const static double wheelCircumference = wheelDiameter * M_PI;
        public: static std::vector<long> CartesianToPolar(std::vector<long> componentVector)
        {
            //NOTE: ALL THESE ARE ALL IN RADIANS NOT DEGREES
            if(componentVector.size() != 2)
            {
                std::cerr << "vectorMath CartesianToPolar warning: Vector size is " + componentVector.size() << std::endl;
            }
            //Initializing polar vector with two elements
            double xComponent = componentVector[0];
            double yComponent = componentVector[1];
            std::vector<long> polarVector (2, 0);
            //Sets first element of vector to length of vector
            polarVector[0] = sqrt(pow(xComponent, 2.0) + pow(yComponent, 2.0));
            //Sets second element to angle
            polarVector[1] = atan(yComponent/xComponent);
            return polarVector;
        }

        public: static std::vector<double> PolarToCartesian(std::vector<double> polarVector)
        {
            //NOTE: THESE ARE ALL IN RADIANS AND NOT DEGREES
            if(polarVector.size() != 2)
            {
                std::cerr << "vectorMath PolarToCartesian warning: Vector size is " + polarVector.size() << std::endl;
            }
            double polarMagnitude = polarVector[0];
            double polarAngle = polarVector[1];
            std::vector<double> componentVector (2,0);
            componentVector[0] = polarMagnitude * cos(polarAngle);
            componentVector[1] = polarMagnitude * sin(polarAngle);
        }
        public: static void ScaleVector(std::vector<double> &cartesianVector, double scalar)
        {
            for(int i = 0; i < cartesianVector.size(); i++)
                cartesianVector[i] *= scalar;
        }    

        public: static double AngleToDistance(double angle)
        {
            return (angle / 360.0) * wheelCircumference;
        }

        public: static double RadiansToDegrees(double radians)
        {
            return radians * 180.0 / M_PI;
        }

        public: static double DegreesToRadians(double degrees)
        {
            return degrees * M_PI / 180.0;
        }
    };