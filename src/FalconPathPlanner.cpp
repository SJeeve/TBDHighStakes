#include <vector>
#include <cmath>
#include <list>
#include <iostream>
#include <math.h>
#include <vex.h>
#include <bits/stdc++.h>
#include <cmath>

FalconPathPlanner::FalconPathPlanner()
{

}
FalconPathPlanner::FalconPathPlanner(std::vector<std::vector<double>> path)
{
    if(path.size() > 1)
    {
        origPath = doubleVectorCopy(path);
        //Don't mess with
        //Except pathBeta that needs to be tuned on a per path basis ideally
        pathBeta = 0.8f;
        pathAlpha = 1 - pathBeta;
        pathTolerance = 0.001f;

        velocityAlpha = 0.1f;
        velocityBeta = 0.3f;
        velocityTolerance = 0.001;
    }
    
}



std::vector<std::vector<double>> FalconPathPlanner::doubleVectorCopy(std::vector<std::vector<double>> vector)
{
    std::vector<std::vector<double>> temp;

    for(int i = 0; i < vector.size(); i++)
    {
        temp.push_back(std::vector<double> {});

        for(int j = 0; j < vector[i].size(); j++)
        {
            temp[i].push_back(vector[i][j]);
        }
    }
    return temp;
}

std::vector<std::vector<double>> FalconPathPlanner::inject(std::vector<std::vector<double>> orig, int numToInject)
{
    std::vector<std::vector<double>> morePoints;

    int index = 0;
    for (int i = 0; i < orig.size() + ((numToInject) * (orig.size() - 1)); i++)
    {
        morePoints.push_back(std::vector<double> {0,0});
    }
    for(int i = 0; i < orig.size() - 1; i++)
    {
        morePoints[index][0] = orig[i][0];
        morePoints[index][1] = orig[i][1];
        index++;
        for(int j=1; j<numToInject + 1; j++)
        {
            //Calculate intermediate x points between j and j+1 original points
            morePoints[index][0] = j*((orig[i+1][0]-orig[i][0])/(numToInject+1)) + orig[i][0];
            //calculate intermediate y points between j and j+1 original points
            morePoints[index][1] = j*((orig[i+1][1])/(numToInject+1))+orig[i][1];
            index++;
        }
    }
    morePoints[index][0] = orig[orig.size()-1][0];
    morePoints[index][1] = orig[orig.size()-1][1];
    index++;

    return morePoints;
}

std::vector<std::vector<double>> FalconPathPlanner::smoother/*I hardly know her*/(std::vector<std::vector<double>> path, double weight_data, double weight_smooth, double tolerance)
{
    std::vector<std::vector<double>> newPath = doubleVectorCopy(path);


    double change = tolerance;
    while(change >= tolerance)
    {
        change = 0.0;
        for(int i = 1; i < path.size() - 1; i++)
        {
            for(int j = 0; j < path[i].size(); j++)
            {
                double aux = newPath[i][j];
                newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
                change += std::abs(aux - newPath[i][j]);
            }
        }
    }
    return newPath;
}

std::vector<std::vector<double>> FalconPathPlanner::nodeOnlyWayPoints(std::vector<std::vector<double>> path)
{
    std::vector<std::vector<double>> li;
    std::vector<std::vector<double>>* liPtr = &li;
    li.push_back(path[0]);

    for(int i = 1; i < path.size() - 2; i++)
    {
        double vector1 = atan2((path[i][1]-path[i-1][1]), path[i][0]-path[i-1][0]);
        double vector2 = atan2((path[i+1][1]-path[i][1]), path[i+1][0]-path[i][0]);

        if(std::abs(vector2 - vector1)>=0.01)
            li.push_back(path[i]);
    }
    li.push_back(path[path.size()-1]);

    std::vector<std::vector<double>> temp;
    for(int i = 0; i < li.size(); i++)
    {
        temp.push_back(std::vector<double> {});
        temp[i].push_back(li[i][0]);
        temp[i].push_back(li[i][1]);
    }

    delete liPtr;
    return temp;
}

std::vector<std::vector<double>> FalconPathPlanner::velocity(std::vector<std::vector<double>> smoothPath, double timeStep)
{
    std::vector<double> dxdt;
    std::vector<double> dydt;

    std::vector<double>* dxdtPtr = &dxdt;
    std::vector<double>* dydtPtr = &dydt;

    std::vector<std::vector<double>> velocity;
    for (int i = 0; i < smoothPath.size(); i++)
    {
        dxdt.push_back(0);
        dydt.push_back(0);
        velocity.push_back(std::vector<double> {0,0});
        heading.push_back(std::vector<double> {0,0});
    }
    dxdt[0] = 0;
    dydt[0] = 0;
    velocity[0][0] = 0;
    velocity[0][1] = 0;

    heading[0][1] = 0;


    for(int i = 1; i < smoothPath.size(); i++)
    {
        dxdt[i] = (smoothPath[i][0]-smoothPath[i-1][0])/timeStep;
        dydt[i] = (smoothPath[i][1]-smoothPath[i-1][1])/timeStep;

        velocity[i][0]=velocity[i-1][0]+timeStep;
        heading[i][0]=heading[i-1][0] + timeStep;

        velocity[i][1] = std::sqrt(std::pow(dxdt[i],2) + std::pow(dydt[i],2));
    }

    delete dxdtPtr;
    delete dydtPtr;
    return velocity;
}

std::vector<std::vector<double>> FalconPathPlanner::velocityFix(std::vector<std::vector<double>> smoothVelocity, std::vector<std::vector<double>> origVelocity, double tolerance)
{
    std::vector<double> difference = errorSum(origVelocity, smoothVelocity);
    std::vector<double>* differencePtr = &difference;
    
    std::vector<std::vector <double>> fixVel;

    for(int i = 0; i < smoothVelocity.size(); i++)
    {
        fixVel.push_back(std::vector<double> {0,0});
        fixVel[i][0] = smoothVelocity[i][0];
        fixVel[i][1] = smoothVelocity[i][1];
    }

    double increase = 0;
    while(std::abs(difference[difference.size()-1])      > tolerance)
    {
        increase = difference[difference.size()-1]/1/50;

        for(int i = 1; i < fixVel.size() - 1; i++)
        {       
            fixVel[i][1] = fixVel[i][1] - increase;
        }
        difference = errorSum(origVelocity, fixVel);
    }

    delete differencePtr;
    return fixVel;
}

std::vector<double> FalconPathPlanner::errorSum(std::vector<std::vector<double>> origVelocity, std::vector<std::vector<double>> smoothVelocity)
{
    std::vector<double> tempOrigDist;
    std::vector<double> tempSmoothDist;
    std::vector<double> difference;

    std::vector<double>* tempOrigDistPtr = &tempOrigDist;
    std::vector<double>* tempSmoothDistPtr = &tempSmoothDist;
    for (int i = 0; i < origVelocity.size(); i++)
    {
        tempOrigDist.push_back(0);
        tempSmoothDist.push_back(0);
        difference.push_back(0);
    }
    double timeStep = origVelocity[1][0]-origVelocity[0][0];

    //copy first elements
    tempOrigDist[0] = origVelocity[0][1];
    tempSmoothDist[0] = smoothVelocity[0][1];

    for(int i = 1; i < origVelocity.size(); i++)
    {
        tempOrigDist[i] = origVelocity[i][1]*timeStep + tempOrigDist[i-1];
        tempSmoothDist[i] = smoothVelocity[i][1]*timeStep + tempSmoothDist[i-1];

        difference[i] = tempSmoothDist[i]-tempOrigDist[i];
    }

    delete tempOrigDistPtr;
    delete tempSmoothDistPtr;

    return difference;
}

std::vector<int> FalconPathPlanner::injectionCounter2Steps(double numNodeOnlyPoints, double maxTimeToComplete, double timeStep){
    int first = 0;
    int second = 0;
    int third = 0;

    double oldPointsTotal = 0;
    numFinalPoints = 0;

    std::vector<int> ret;
    double totalPoints = maxTimeToComplete/timeStep;
    
    if(totalPoints < 100)
    {
        double pointsFirst = 0;
        double pointsTotal = 0;

        for(int i = 4; i <= 6; i++)
        {
            for(int j = 1; j <=8; j++)
            {
                pointsFirst = i * (numNodeOnlyPoints-1) + numNodeOnlyPoints;
                pointsTotal = (j*(pointsFirst-1)+pointsFirst);

                if(pointsTotal<=totalPoints && pointsTotal>oldPointsTotal)
                {
                    first=i;
                    second=j;
                    numFinalPoints=pointsTotal;
                    oldPointsTotal=pointsTotal;
                }
            }
        }
        ret = {first, second, third};
    } else {
        double pointsFirst = 0;
        double pointsSecond = 0;
        double pointsTotal = 0;

        for(int i = 1; i <= 5; i++)
        {
            for(int j=1; j<=8;j++)
            {
                for(int k = 1; k < 8; k++)
                {
                    pointsFirst = i * (numNodeOnlyPoints-1) + numNodeOnlyPoints;
                    pointsSecond = j*(pointsFirst-1)+pointsFirst;
                    pointsTotal = k*(pointsSecond-1)+pointsSecond;

                    if(pointsTotal <= totalPoints)
                    {
                        first = i;
                        second = j;
                        third = k;
                        numFinalPoints=pointsTotal;
                    }
                }
            }
        }
        ret = {first, second, third};
    }

    return ret;
}

void FalconPathPlanner::leftRight(std::vector<std::vector<double>> smoothPath, double robotTrackWidth)
{
    std::vector<std::vector<double>> leftPath;
    std::vector<std::vector<double>> rightPath;

    std::vector<std::vector<double>> gradient;
    std::vector<std::vector<double>>* leftPathPtr = &leftPath;
    std::vector<std::vector<double>>* rightPathPtr = &rightPath;

    std::vector<std::vector<double>>* gradientPtr = &gradient;
    gradient.push_back(std::vector<double> {0, 0});
    leftPath.push_back(std::vector<double> {0, 0});
    rightPath.push_back(std::vector<double> {0, 0});

    for(int i = 0; i < smoothPath.size() - 1; i++)
    {
        gradient.push_back(std::vector<double> {0,0});
        leftPath.push_back(std::vector<double> {0,0});
        rightPath.push_back(std::vector<double> {0,0});
        gradient[i][1] = atan2(smoothPath[i+1][1] - smoothPath[i][1], smoothPath[i+1][0] - smoothPath[i][0]);

    }

    gradient[gradient.size() - 1][1] = gradient[gradient.size()-2][1];

    for(int i = 0; i < gradient.size(); i++)
    {
        leftPath[i][0] = (robotTrackWidth/2 * cos(gradient[i][1] + M_PI/2)) + smoothPath[i][0];
        leftPath[i][1] = (robotTrackWidth/2 * sin(gradient[i][1] + M_PI/2)) + smoothPath[i][1];   

        rightPath[i][0] = robotTrackWidth / 2 * cos(gradient[i][1] - M_PI / 2) + smoothPath[i][0];
        rightPath[i][1] = robotTrackWidth / 2 * sin(gradient[i][1] - M_PI / 2) + smoothPath[i][1];

         double deg = vectorMath::RadiansToDegrees(gradient[i][1]);
         if(i > 0)
         {
            if((deg-gradient[i-1][1]) > 180)
                gradient[i][1] = -360 + deg;
            if((deg-gradient[i-1][1]) < -180)
                gradient[i][1] = 360 + deg;
         }
    }
    this->heading.clear();
    this->leftPath.clear();
    this->rightPath.clear();
    
    std::copy(heading.begin(), heading.end(), back_inserter(this->heading));
    std::copy(leftPath.begin(), leftPath.end(), back_inserter(this->leftPath));
    std::copy(rightPath.begin(), rightPath.end(), back_inserter(this->rightPath));

    delete leftPathPtr;
    delete rightPathPtr;
    delete gradientPtr;
}

std::vector<double> FalconPathPlanner::getXVector(std::vector<std::vector<double>> arr)
{
    std::vector<double> temp;
    for(int i = 0; i < arr.size(); i++)
    {
        temp.push_back(arr[i][0]);
    }
    return temp;
}

std::vector<double> FalconPathPlanner::getYVector(std::vector<std::vector<double>> arr)
{
    std::vector<double> temp;
    for(int i = 0; i < arr.size(); i++)
    {
        temp.push_back(arr[i][1]);
    }
    return temp;
}

std::vector<std::vector<double>> FalconPathPlanner::transposeVector(std::vector<std::vector<double>> arr)
{
    std::vector<std::vector<double>> temp;

    for(int i = 0; i < arr.size(); i++)
    {
        for(int j = 0; j < arr[i].size(); j++)
        {
            temp[i][j] = arr[j][i];
        }
    }
    return temp;
}

void FalconPathPlanner::setPathAlpha(double alpha)
{
    pathAlpha = alpha;
}

void FalconPathPlanner::setPathBeta(double beta)
{
    pathBeta = beta;
}

void FalconPathPlanner::setPathTolereance(double tolerance)
{
    pathTolerance = tolerance;
}

void FalconPathPlanner::calculate(double totalTime, double timeStep, double robotTrackWidth)
{
    nodeOnlyPath = nodeOnlyWayPoints(origPath);

    std::vector<int> inject = FalconPathPlanner::injectionCounter2Steps(nodeOnlyPath.size(), totalTime, timeStep);
    for(int i = 0; i < inject.size(); i++)
    {   
        if(i==0)
        {
            smoothPath = FalconPathPlanner::inject(nodeOnlyPath, inject[0]);
            smoothPath = FalconPathPlanner::smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);

        } else {

            smoothPath = FalconPathPlanner::inject(smoothPath, inject[i]);
            smoothPath = FalconPathPlanner::smoother(smoothPath, 0.1, 0.3, 0.0000001);
        }
    }

    leftRight(smoothPath, robotTrackWidth);
    std::cout << "there" << std::endl;
    origCenterVelocity = FalconPathPlanner::velocity(smoothPath, timeStep);
    origLeftVelocity = FalconPathPlanner::velocity(leftPath, timeStep);
    origRightVelocity = FalconPathPlanner::velocity(rightPath, timeStep);

    smoothCenterVelocity = FalconPathPlanner::doubleVectorCopy(origCenterVelocity);
    smoothLeftVelocity = FalconPathPlanner::doubleVectorCopy(origLeftVelocity);
    smoothRightVelocity = FalconPathPlanner::doubleVectorCopy(origRightVelocity);

    smoothCenterVelocity[smoothCenterVelocity.size() - 1][1] = 0.0;
    smoothLeftVelocity[smoothLeftVelocity.size() - 1][1] = 0.0;
    smoothRightVelocity[smoothRightVelocity.size() - 1][1] = 0.0;

    smoothCenterVelocity = FalconPathPlanner::smoother(smoothCenterVelocity, velocityAlpha, velocityBeta, velocityTolerance);
    smoothLeftVelocity = FalconPathPlanner::smoother(smoothLeftVelocity, velocityAlpha, velocityBeta, velocityTolerance);
    smoothRightVelocity = FalconPathPlanner::smoother(smoothRightVelocity, velocityAlpha, velocityBeta, velocityTolerance);
    double tolerance = 0.0000001;
    smoothCenterVelocity = FalconPathPlanner::velocityFix(smoothCenterVelocity, origCenterVelocity, tolerance);
    smoothLeftVelocity = FalconPathPlanner::velocityFix(smoothLeftVelocity, origLeftVelocity, tolerance);
    smoothRightVelocity = FalconPathPlanner::velocityFix(smoothRightVelocity, origRightVelocity, tolerance);

}