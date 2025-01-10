

#ifndef PATHING_H
#define PATHING_H
#include <vex.h>
#include <vector>
#include <cmath>
#include <list>
#include <iostream>
#include <math.h>


class Pathing{
    public:
        FalconPathPlanner smoothedPath;
        std::vector<std::vector<double>> path;
        double spacing = 6.0f;
        std::vector<double> curvatureList;
        //Think of this like a parametric equation
        std::vector<double> targetVelocities;
        //Keep this between 1-5
        double maxTurnSpeed = 2;    
        std::vector<double> distanceBetweenPoints;
        double maxVelocity = 5;
        double maxAcceleration = 5;
        int lastClosestPoint = 0;
        double lastOutput = 0;
        double last_time;
        double lookaheadDistance;
        std::vector<double> lookaheadPoint;
        double lastLookAheadPointIndex = 0;
        double VelocityConstant;
        double AccelerationConstant;
        double FeedbackConstant;

        double totalTime;
        double timeStep;

    Pathing(std::vector<std::vector<double>> keyPoints);
    Pathing(std::vector<std::vector<double>> keyPoints, double _spacing, double maxV, double maxA);
    std::vector<std::vector<double>> PopulatePath(std::vector<std::vector<double>> keyPoints);
    void populateDistanceData();
    void SettingUpPaths();
    double calculateCurvature(std::vector<double> point1, std::vector<double> point2, std::vector<double> point3);
    void populateCurvatureData();
    void findClosestPoint(std::vector<double> robotPosition);
    void populateVelocityData();
    double distanceBetweenTwoPoints(std::vector<double> point1, std::vector<double> point2);
    double rateLimiter(double);
    double calculateIntersectionPoint(double, double, double);
    std::vector<double> calculateLookAheadPoint(std::vector<double> directionVector, std::vector<double> rayStart, double intersection);
    void UpdateLookAheadPoint();
    bool validLookAheadPoint(int);
    void CalculatingWheelVelocities();
    void Path();
    void UpdatePosition();
    static void resetRotationSensors();
    void ConstructFalconPath();
};
//I have no idea what I'm doing on Skibidi
#endif PATHING_H