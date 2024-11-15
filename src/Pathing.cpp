#include <vector>
#include <cmath>
#include <list>
#include <iostream>
#include <math.h>
#include <vex.h>


    Pathing::Pathing(std::list<std::vector<float>> keyPoints){
        PopulatePath(keyPoints);
    }

    Pathing::Pathing(std::list<std::vector<float>> keyPoints, float _spacing)
    {
        spacing = _spacing;
        PopulatePath(keyPoints);
    }

    void Pathing::PopulatePath(std::list<std::vector<float>> keyPoints)
    {
        for(auto i : keyPoints)
            path.push_back(i);
        
        std::cout << "Initial path initialized: " << ' ';
        std::cout << path.size() << ' ';
        std::vector<std::vector<float>> newPoints;
        for(int i = 0; i < path.size() - 1; i++)
        {
            std::vector<float> lineVector = vectorMath::SubtractVectors(path[i], path[i + 1]);
            int pointsThatFit = std::ceil(vectorMath::MagnitudeOfVector(lineVector) / spacing);
            lineVector = vectorMath::ScaleVector(vectorMath::NormalizeVector(lineVector), spacing);
            for(int j = 0; j < pointsThatFit; j++)
            {
                newPoints.push_back(vectorMath::AddVectors(path[i], vectorMath::ScaleVector(lineVector, i)));
            }
        }
        newPoints.push_back(path[path.size() - 1]);
        path = newPoints;
    }