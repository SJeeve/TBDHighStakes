#include <vex.h>
#include <vector>
#include <iostream>

using namespace vex;
using namespace std;

    class VectorMath{
        public:
        static std::vector<long> ComponentToPolar(std::vector<long> componentVector)
        {
            if(componentVector.size() != 2)
            {
                std::cerr << "vectorMath ComponentToPolar warning: Vector size is " + componentVector.size() << std::endl;
            }
            //Initializing polar vector with two elements
            long xComponent = componentVector[0];
            long yComponent = componentVector[1];
            std::vector<long> polarVector (2, 0);
            //Sets first element of vector to length of vector
            polarVector[0] = sqrt(pow(xComponent, 2.0) + pow(yComponent, 2.0));
            //Sets second element to angle
            polarVector[1] = atan(yComponent/xComponent);
            return polarVector;
        }

        static std::vector<long> PolarToComponent(std::vector<long> polarVector)
        {
            if(polarVector.size() != 2)
            {
                std::cerr << "vectorMath PolarToComponent warning: Vector size is " + polarVector.size() << std::endl;
            }
            long polarMagnitude = polarVector[0];
            long polarAngle = polarVector[1];
            std::vector<long> componentVector (2,0);
            componentVector[0] = polarMagnitude * cos(polarAngle);
            componentVector[1] = polarMagnitude * sin(polarAngle);
        }
    };