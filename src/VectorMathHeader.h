#ifndef VECTORMATHHEADER_H
#define VECTORMATHHEADER_H

#include <vector>
#include <iostream>
#include <cmath>
namespace vectormath{
class VectorMath {
    public:
        // Wheel circumference variable (to be defined and set in the implementation)
        static constexpr float wheelCircumference = 2.75f;
        // Convert Cartesian coordinates to Polar coordinates
        static void CartesianToPolar(std::vector<float> &componentVector);

        // Add two vectors
        static void AddVectors(std::vector<float> &holderVector, const std::vector<float> &addedVector);

        // Rotate a polar vector by a given angle in radians
        static void RotateVectorAddition(std::vector<float> &polarVector, float radians);

        // Convert Polar coordinates to Cartesian coordinates
        static void PolarToCartesian(std::vector<float> &polarVector);

        // Scale a Cartesian vector by a scalar
        static void ScaleVector(std::vector<float> &cartesianVector, float scalar);

        // Convert an angle in degrees to the corresponding distance based on wheel circumference
        static float AngleToDistance(float angle);

        // Convert radians to degrees
        static float RadiansToDegrees(float radians);

        // Convert degrees to radians
        static float DegreesToRadians(float degrees);
    };
}
#endif // VECTORMATH_H