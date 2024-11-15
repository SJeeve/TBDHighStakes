

#ifndef PATHING_H
#define PATHING_H
#include <vector>
#include <cmath>
#include <list>
#include <iostream>
#include <math.h>
#include <vex.h>
class Pathing{
    public:
        std::vector<std::vector<float>> path;
        float spacing = 6.0f;
    Pathing(std::list<std::vector<float>> keyPoints);
    Pathing(std::list<std::vector<float>> keyPoints, float _spacing);
    void PopulatePath(std::list<std::vector<float>> keyPoints);

}; //Why does this need a semicolon :(
#endif 