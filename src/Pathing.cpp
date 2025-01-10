#include <vector>
#include <cmath>
#include <list>
#include <iostream>
#include <math.h>
#include <vex.h>
using namespace vex;

    void Pathing::ConstructFalconPath()
    {
        totalTime = 8;
        timeStep = 0.1;
        smoothedPath.calculate(totalTime, timeStep, trackWidth);
    }
    /// @brief Constructs a pathing object with a vector of keypoints. 
    /// @param keyPoints 
    Pathing::Pathing(std::vector<std::vector<double>> keyPoints){
        path = PopulatePath(keyPoints);
        smoothedPath = FalconPathPlanner(path);
        last_time = (double)Brain.timer(sec);
        
        //During tuning keep velocity constant and then fine tune it until the target and measured velocities match up when the robot is traveling a constant speed
        VelocityConstant = 1/maxVelocity;
        AccelerationConstant = 0.002;
        FeedbackConstant = 0.001;
        SettingUpPaths();
        ConstructFalconPath();
    }

    /// @brief Sets up the path data 
    void Pathing::SettingUpPaths()
    {
        smoothedPath.calculate(totalTime, timeStep, trackWidth);
        populateCurvatureData();
        populateDistanceData();
        populateVelocityData();
    }

    Pathing::Pathing(std::vector<std::vector<double>> keyPoints, double _spacing, double maxV, double maxA)
    {
        maxVelocity = maxV;
        maxAcceleration = maxA;
        spacing = _spacing;
        path = PopulatePath(keyPoints);
        smoothedPath = FalconPathPlanner(path);
        //These need to be fine tuned
        VelocityConstant = 1/maxV;
        //During tuning keep velocity constant and then fine tune it until the target and measured velocities match up when the robot is traveling a constant speed
        AccelerationConstant = 0.002;
        FeedbackConstant = 0.001;
        SettingUpPaths();
        ConstructFalconPath();
    }
    /// @brief Updates the position of the robot. A little redundant because position also has this function but this makes it a bit cleaner to call in this class
    void Pathing::UpdatePosition()
    {
        position.positionSensing::UpdatePosition();
        resetRotationSensors();
    }
    void Pathing::resetRotationSensors()
    {
        leftTrackingWheel.resetPosition();
        rightTrackingWheel.resetPosition();
        backTrackingWheel.resetPosition();
    }

    /// @brief This injects more points into the list of important points. This will accentuate extremes in the path. Don't use if you want a smoother path
    /// @param keyPoints 
    /// @return the new path
    std::vector<std::vector<double>> Pathing::PopulatePath(std::vector<std::vector<double>> keyPoints)
    {
        for(auto i : keyPoints)
            path.push_back(i);

        std::vector<std::vector<double>> newPoints;
        for(int i = 0; i < path.size() - 1; i++)
        {
            std::vector<double> lineVector = vectorMath::SubtractVectors(path[i], path[i + 1]);
            int pointsThatFit = std::ceil(vectorMath::MagnitudeOfVector(lineVector) / spacing);
            lineVector = vectorMath::ScaleVector((vectorMath::NormalizeVector(lineVector)), spacing);
            for(int j = 0; j < pointsThatFit; j++)
            {
                newPoints.push_back(vectorMath::AddVectors(path[i], vectorMath::ScaleVector(lineVector, i)));
            }
        }
        newPoints.push_back(path[path.size() - 1]);
        path = newPoints;
        return path;
    }
    /// @brief Calculates the needed wheel velocities to follow the path. Updates every 10 msec
    void Pathing::Path()
    {
        while(true){
            UpdatePosition();
            findClosestPoint(position.currentPosition);
            UpdateLookAheadPoint();
            CalculatingWheelVelocities();
            vex::wait(10, msec);
        }
    }
    //Calculates the target velocities for each point along the path
    void Pathing::populateVelocityData()
    {
        for(int i = 0; i < smoothedPath.smoothPath.size(); i++)
        {
            targetVelocities[i] = vectorMath::minf(maxVelocity, maxVelocity * curvatureList[i]);
        }
        targetVelocities[targetVelocities.size() - 1] = 0;
        for(int i = smoothedPath.smoothPath.size() - 2; i >= 0; i--)
        {
            double distance = distanceBetweenTwoPoints(smoothedPath.smoothPath[i+1], smoothedPath.smoothPath[i]);
            targetVelocities[i] = vectorMath::minf(targetVelocities[i], std::sqrt(pow(targetVelocities[i+1],2) + 2 * maxAcceleration * distance));
        }
    }
    /// @brief Limits the amount of change in the wheels by how much time has passed since the last call
    /// @param limiting 
    /// @return 
    double Pathing::rateLimiter(double limiting)
    {
        double current_time = (double)Brain.timer(sec);
        double difference;
        difference = current_time - last_time;

        double maxChange = difference * maxAcceleration;
        double output = vectorMath::Clampf(limiting - lastOutput, -1 * maxChange, maxChange);
        lastOutput = output;
        last_time = current_time;

        return output;
    }

    /// @brief Finds the closest point relative the robot's current position
    /// @param robotPosition 
    void Pathing::findClosestPoint(std::vector<double> robotPosition)
    {
        double closestDistance = distanceBetweenTwoPoints(robotPosition, smoothedPath.smoothPath[lastClosestPoint]);
        for(int i = lastClosestPoint; i < smoothedPath.smoothPath.size(); i++)
        {
            double distance = distanceBetweenTwoPoints(robotPosition, smoothedPath.smoothPath[i]);
            if(distance < closestDistance)
            {
                closestDistance = distance;
                lastClosestPoint = i;
            }
        }
    }    
    /// @brief Populates vector with information containing the distance between each point and the point following it
    void Pathing::populateDistanceData()
    {
        distanceBetweenPoints.push_back(0);
        for(int i = 1; i < smoothedPath.smoothPath.size(); i++)
        {
            distanceBetweenPoints[i] = distanceBetweenPoints[i-1] + distanceBetweenTwoPoints(smoothedPath.smoothPath[i], smoothedPath.smoothPath[i - 1]);
        }
    }
    /// @brief Returns the distance between two points
    /// @param point2 
    /// @param point1 
    /// @return 
    double Pathing::distanceBetweenTwoPoints(std::vector<double> point2, std::vector<double> point1)
    {
        //
        return std::sqrt(pow((point2[0] - point1[0]),2) + pow((point2[1] - point1[1]),2));
    }
    /// @brief Populates vector with data containing the curvature at each point in the path
    void Pathing::populateCurvatureData()
    {
        curvatureList.push_back(0);
        for(int i = 1; i < smoothedPath.smoothPath.size() - 1; i++)
        {
            curvatureList.push_back(calculateCurvature(smoothedPath.smoothPath[i-1], smoothedPath.smoothPath[i], smoothedPath.smoothPath[i+1]));
        }   
        curvatureList.push_back(0);
    }

    /// @brief Calculates the curvature between three points
    /// @param point1 
    /// @param point2 
    /// @param point3 
    /// @return the curvature
    double Pathing::calculateCurvature(std::vector<double> point1, std::vector<double> point2, std::vector<double> point3)
    {
        
        double x1 = point1[0];
        double x2 = point2[0];
        double x3 = point3[0];
        double y1 = point1[1];
        double y2 = point2[1];
        double y3 = point3[1];
        double x1p = std::pow(point1[0],2);
        double x2p = std::pow(point2[0],2);
        double x3p = std::pow(point3[0],2);
        double y1p = std::pow(point1[1],2);
        double y2p = std::pow(point2[1],2);
        double y3p = std::pow(point3[1],2);
        if(x1 == x2)
            x1 += 0.001;
        double k1 = 0.5 * (x1p + y1p - x2p - y2p)/(x1 - x2);
        double k2 = (y1 - y2)/(x1-x2);
        double b = 0.5 * (x2p - 2 * x2 * k1 + y2p - x3p + 2 * x3 * k1 - y3p)/(x3 * k2 - y3 + y2 -x2 * k2);
        double a = k1 - k2 * b;

        double output = 1/std::sqrt(std::pow((x1 - a),2) + std::pow(y1 - b, 2)); 
        //If the output is nan that means we have a straight line at this point, and the curvature is 1/infinity, which is 0
        if(std::isnan(output))
        {
            return 0;
        } else {
            return output;
        }
    
    }
    /// @brief Calculates the needed wheel velocities to follow path. See definition of function for more information
    void Pathing::CalculatingWheelVelocities()
    {

        //curvature of robot arc is 2x/L^2
        //L is the lookahead distance and x is the horizontal distance from an imaginary line representing the robot and the lookahead point

        //I know one letter variables are bad practice but I can't think of particulary helpful names rn so Im just not gonna + I did the math like this
        double Rx = position.currentPosition[0];
        double Ry = position.currentPosition[1];
        double Lx = lookaheadPoint[0];
        double Ly = lookaheadPoint[1];
        double theta = position.lastResetGlobalOrientation;
    
        double a = -1 * std::tan(theta);
        double c = a * -1 * Rx - Ry;

        double curvature = 2 * std::abs(a * Lx + Ly + c) / std::sqrt(std::pow(a, 2) + 1);
        curvature /= lookaheadDistance * lookaheadDistance;


        double side = ((std::sin(theta)) * (Lx - Rx) - (std::sin(theta)) * (Ly - Ry));
        side = static_cast<double>(std::signbit(side));
        side = side == 1 ? 1: -1;
        curvature *= side;
        double leftWheelVelocity = (rateLimiter(targetVelocities[lastClosestPoint]) + targetVelocities[lastClosestPoint]) * (2 + curvature * trackWidth)/2;
        double rightWheelVelocity = (rateLimiter(targetVelocities[lastClosestPoint]) + targetVelocities[lastClosestPoint]) * (2 - curvature * trackWidth)/2;

        double FeedForwardLeft = VelocityConstant * leftWheelVelocity + AccelerationConstant * (2 + curvature * trackWidth)/2;
        double FeedForwardRight = VelocityConstant * rightWheelVelocity + AccelerationConstant * (2 - curvature * trackWidth)/2;
        //Need to change feedBack to use a velocity measurement consistent with the measurement used in the max velocity variable
        double FeedBackLeft = FeedbackConstant * (leftWheelVelocity - (wheelTravel * leftFront.velocity(rpm)));
        double FeedBackRight = FeedbackConstant * (rightWheelVelocity - (wheelTravel * rightFront.velocity(rpm)));

        leftWheelVelocity = FeedForwardLeft + FeedBackLeft;
        rightWheelVelocity = FeedForwardRight + FeedBackRight;
        //Tbh Idk what unit this should use
        leftDriveSmart.spin(vex::forward, leftWheelVelocity, volt);
        rightDriveSmart.spin(vex::forward, rightWheelVelocity, volt);
    }
    /// @brief Updates the look ahead point to be relevant to the current position
    void Pathing::UpdateLookAheadPoint(){
        for(int i = lastLookAheadPointIndex; i < smoothedPath.smoothPath.size() - 1; i++)
        {
            double discriminant;
            std::vector<double> rayStart = smoothedPath.smoothPath[i];
            std::vector<double> rayEnd = smoothedPath.smoothPath[i + 1];
            std::vector<double> directionVector = vectorMath::SubtractVectors(rayStart, rayEnd);
            std::vector<double> centerCircleVector = vectorMath::SubtractVectors(position.positionSensing::GetPosition(), rayStart);
            double a = vectorMath::DotVectors(directionVector, directionVector);
            double b = 2 * vectorMath::DotVectors(centerCircleVector, directionVector);
            double c = vectorMath::DotVectors(centerCircleVector, centerCircleVector) - lookaheadDistance * lookaheadDistance;

            discriminant = b * b - 4 * a * c;
            if(discriminant > 0)
            {
                double intersection = calculateIntersectionPoint(discriminant, a, b);
                if(intersection + i > lastLookAheadPointIndex)
                {
                    lastLookAheadPointIndex = intersection + i;
                    lookaheadPoint = calculateLookAheadPoint(directionVector, rayStart, intersection);
                    return;
                }
            }

        }
    }

    double Pathing::calculateIntersectionPoint(double discriminant, double a, double b)
    {
        if(discriminant < 0)
        {
            //no intersection
        } else {
            discriminant = std::sqrt(discriminant);
            double t1 = (-1 * b - discriminant) / (2*a);
            double t2 = (-1 * b + discriminant) / (2*a);

            if(t1 >= 0 &&  t1 <= 1)
            {
                return t1;
            }
            if(t2 >= 0 && t2 <= 1)
            {
                return t2;
            }
            //no intersection
        }
        return -1;
    }

    std::vector<double> Pathing::calculateLookAheadPoint(std::vector<double> directionVector, std::vector<double> rayStart, double intersection)
    {
        std::vector<double> lookAheadPoint = vectorMath::AddVectors(vectorMath::ScaleVector(directionVector, intersection), rayStart);
        return lookAheadPoint;
    }

    