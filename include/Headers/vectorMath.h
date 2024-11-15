#ifndef VECTORMATHHEADER_H
#define VECTORMATHHEADER_H

#include <vector>
#include <iostream>
#include <cmath>

class vectorMath {
    public:
        // Wheel circumference variable (to be defined and set in the implementation)
        static constexpr float wheelCircumference = 2.75f;
        // Convert Cartesian coordinates to Polar coordinates
        static void CartesianToPolar(std::vector<float> &componentVector);

        // Add two vectors
        static std::vector<float> AddVectors(std::vector<float> holderVector, std::vector<float> addedVector);

        //Subtract two vectors, second one is the one you subtract from
        static std::vector<float> SubtractVectors(std::vector<float> &subtractingVector, std::vector<float> subtractedVector);
        // Rotate a polar vector by a given angle in radians
        static void RotateVectorAddition(std::vector<float> &polarVector, float radians);

        // Convert Polar coordinates to Cartesian coordinates
        static void PolarToCartesian(std::vector<float> &polarVector);

        //Normalizes a vector (gives the line representing the vector a length of one, but keeps direction)
        static std::vector<float> NormalizeVector(std::vector<float> normalizedVector);

        // Scale a Cartesian vector by a scalar
        static std::vector<float> ScaleVector(std::vector<float> cartesianVector, float scalar);

        // Convert an angle in degrees to the corresponding distance based on wheel circumference
        static float AngleToDistance(float angle);

        // Convert radians to degrees
        static float RadiansToDegrees(float radians);

        // Convert degrees to radians
        static float DegreesToRadians(float degrees);

        static float Clampf(float value, float minimum, float maximum);

        static float MagnitudeOfVector(std::vector<float> &cartesianVector);
    };
#endif // VECTORMATH_H