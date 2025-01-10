#ifndef FALCONPATH_H
#define FALCONPATH_H
#include <vector>
#include <cmath>
#include <list>
#include <iostream>
#include <math.h>

class FalconPathPlanner{
    public:
    //path variables
    std::vector<std::vector<double>> origPath;
    std::vector<std::vector<double>> nodeOnlyPath;
    std::vector<std::vector<double>> smoothPath;
    std::vector<std::vector<double>> leftPath;
    std::vector<std::vector<double>> rightPath;

    //original velocity
    std::vector<std::vector<double>> origCenterVelocity;
    std::vector<std::vector<double>> origRightVelocity;
    std::vector<std::vector<double>> origLeftVelocity;

    //Smooth velocity
    std::vector<std::vector<double>> smoothCenterVelocity;
    std::vector<std::vector<double>> smoothRightVelocity;
    std::vector<std::vector<double>> smoothLeftVelocity;

    //accumulated heading
    std::vector<std::vector<double>> heading;

    double totalTime;
    double totalDistance;
    double numFinalPoints;

    double pathAlpha;
    double pathBeta;
    double pathTolerance;

    double velocityAlpha;
    double velocityBeta;
    double velocityTolerance;
    FalconPathPlanner();
    FalconPathPlanner(std::vector<std::vector<double>> path);

    std::vector<std::vector<double>> doubleVectorCopy(std::vector<std::vector<double>> vector);

    std::vector<std::vector<double>> inject(std::vector<std::vector<double>> orig, int numToInject);

    std::vector<std::vector<double>> smoother(std::vector<std::vector<double>> path, double weight_data, double weight_smooth, double tolerance);
    
    static std::vector<std::vector<double>> nodeOnlyWayPoints(std::vector<std::vector<double>> path);

    std::vector<std::vector<double>> velocity(std::vector<std::vector<double>> smoothPath, double timeStep);
    
    std::vector<double> errorSum(std::vector<std::vector<double>> origVelocity, std::vector<std::vector<double>> smoothVelocity);
    
    std::vector<std::vector<double>> velocityFix(std::vector<std::vector<double>> smoothVelocity, std::vector<std::vector<double>> origVelocity, double tolerance);
    
    std::vector<int> injectionCounter2Steps(double numNodeOnlyPoints, double maxTimeToComplete, double timeStep);
    
    void leftRight(std::vector<std::vector<double>> smoothPath, double robotTrackWidth);
    
    static std::vector<double> getXVector(std::vector<std::vector<double>> arr);
    
    std::vector<double> getYVector(std::vector<std::vector<double>> arr);
    
    std::vector<std::vector<double>> transposeVector(std::vector<std::vector<double>> arr);
    
    void setPathAlpha(double alpha);
    
    void setPathBeta(double beta);
    
    void setPathTolereance(double tolerance);
    
    void calculate(double totalTime, double timeStep, double robotTrackWidth);
}; 
#endif FALCONPATH_H