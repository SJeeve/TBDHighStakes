#include <iostream>
#include <vex.h>
#include <cmath>

        //Wheel diameter is somewhere between 2.5 and 2.75
        //Needs more testing to get actual value
        
        /// @brief Converts a given cartesian vector to its polar form. 
        /// Note that this function does not return anything as the reference to the vector is passed
        /// @param componentVector 
        void vectorMath::CartesianToPolar(std::vector<double> &componentVector)
        {
            //NOTE: ALL THESE ARE ALL IN RADIANS NOT DEGREES
            //& means pass by reference
            double xComponent = componentVector[0];
            double yComponent = componentVector[1];
            //Sets first element of vector to length of vector
            componentVector[0] = sqrt(pow(xComponent, 2) + pow(yComponent, 2));
            //Sets second element to angle
            componentVector[1] = atan(yComponent/xComponent);

        }
        /// @brief Returns the smaller value between two numbers
        /// @param num1 
        /// @param num2 
        /// @return 
        double vectorMath::minf(double num1, double num2)
        {
            if(num1 < num2)
                return num1;
            return num2;
        }

        /// @brief Adds two vectors together
        /// @param holderVector 
        /// @param addedVector 
        /// @return The resulting vector
        std::vector<double> vectorMath::AddVectors(std::vector<double> holderVector, std::vector<double> addedVector)
        {

            holderVector[0] = holderVector[0] + addedVector[0];
            holderVector[1] = holderVector[1] + addedVector[1];

            return holderVector;
        }

        /// @brief Returns the dot product of two vectors. Vectors are passed by reference to save memory
        /// @param vector1 
        /// @param vector2 
        /// @return The dot product
        double vectorMath::DotVectors(std::vector<double> &vector1, std::vector<double> &vector2)
        {
            double output = 0;
            for(int i = 0; i < vector1.size(); i++)
            {
                output += vector1[i] * vector2[i];
            }
            return output;
        }

        /// @brief Subtracts one vector from another. The first vector is subtracted from the second vector
        /// @param subtractingVector 
        /// @param subtractedVector 
        /// @return The new vector
        std::vector<double> vectorMath::SubtractVectors(std::vector<double> subtractingVector, std::vector<double> subtractedVector)
        {
            for(int i = 0; i < subtractedVector.size(); i++)
                subtractedVector[i] -= subtractingVector[i];
            return subtractedVector;
        }
        /// @brief This normalizes the vector by dividing each component by the vector's magnitude
        /// @param normalizedVector 
        /// @return normalized vector
        std::vector<double> vectorMath::NormalizeVector(std::vector<double> normalizedVector)
        {
            double lengthMultiplier = 1/MagnitudeOfVector(normalizedVector);
            for(int i = 0; i < normalizedVector.size(); i++)
                normalizedVector[i] *= lengthMultiplier;
            return normalizedVector;
        }
        /// @brief Rotates a vector by a given angle in radians
        /// @param polarVector 
        /// @param radians 
        /// @return The new angle
        void vectorMath::RotateVectorAddition(std::vector<double> &polarVector, double radians)
        {
            polarVector[1] += radians;
        }

        /// @brief Clamps a given value between a minimum and maximum 
        /// @param value 
        /// @param minimum 
        /// @param maximum 
        /// @return clamped value
        double vectorMath::Clampf(double value, double minimum, double maximum)
        {
            if(value < minimum)
                return minimum;
            if(value > maximum)
                return maximum;
            return value;
        }

        /// @brief Returns the magnitude/length of a vector
        /// @param cartesianVector 
        /// @return magnitude
        double vectorMath::MagnitudeOfVector(std::vector<double> &cartesianVector)
        {
            return sqrt(pow(cartesianVector[0], 2) + pow(cartesianVector[1], 2));
        }
        
        /// @brief Converts a polar vector into a cartesian vector. Use radians for angle
        /// @param polarVector 
        void vectorMath::PolarToCartesian(std::vector<double> &polarVector)
        {
            //NOTE: THESE ARE ALL IN RADIANS AND NOT DEGREES
            double polarMagnitude = polarVector[0];
            double polarRadians = polarVector[1];
            polarVector[0] = polarMagnitude * cos(polarRadians);
            polarVector[1] = polarMagnitude * sin(polarRadians);
        }

        /// @brief This scales a given vector by a given scalar
        /// @param cartesianVector 
        /// @param scalar 
        /// @return scaled vector
        void vectorMath::ScaleVector(std::vector<double> &cartesianVector, double scalar)
        {
            for(int i = 0; i < cartesianVector.size(); i++)
                cartesianVector[i] *= scalar;

        }    
        /// @brief Returns how far a wheel has traveled base on the change in angle. Angle measured in degrees
        /// @param angle 
        /// @return distance
        double vectorMath::AngleToDistance(double angle)
        {

            return (angle / 360.0) * wheelTravel;
        }

        double vectorMath::RadiansToDegrees(double radians)
        {
            return radians * 180.0 / M_PI;
        }

        double vectorMath::DegreesToRadians(double degrees)
        {
            return degrees * M_PI / 180.0;
        }