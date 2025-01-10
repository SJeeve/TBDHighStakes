#ifndef VECTORMATHHEADER_H
#define VECTORMATHHEADER_H

#include <vector>
#include <iostream>
#include <cmath>

class vectorMath {
    public:

        // Convert Cartesian coordinates to Polar coordinates
        static void CartesianToPolar(std::vector<double> &componentVector);

        // Add two vectors
        static std::vector<double> AddVectors(std::vector<double> holderVector, std::vector<double> addedVector);

        //Subtract two vectors, second one is the one you subtract from
        static std::vector<double> SubtractVectors(std::vector<double> subtractingVector, std::vector<double> subtractedVector);
        // Rotate a polar vector by a given angle in radians
        static void RotateVectorAddition(std::vector<double> &polarVector, double radians);

        // Convert Polar coordinates to Cartesian coordinates
        static void PolarToCartesian(std::vector<double> &polarVector);

        //Normalizes a vector (gives the line representing the vector a length of one, but keeps direction)
        static std::vector<double> NormalizeVector(std::vector<double> normalizedVector);

        // Scale a Cartesian vector by a scalar
        static void ScaleVector(std::vector<double> &cartesianVector, double scalar);

        // Convert an angle in degrees to the corresponding distance based on wheel circumference
        static double AngleToDistance(double angle);
        static double minf(double num1, double num2);
        // Convert radians to degrees
        static double RadiansToDegrees(double radians);

        // Convert degrees to radians
        static double DegreesToRadians(double degrees);

        static double Clampf(double value, double minimum, double maximum);

        static double MagnitudeOfVector(std::vector<double> &cartesianVector);

        static double DotVectors(std::vector<double> &vector1, std::vector<double> &vector2);
    };
#endif // VECTORMATH_H