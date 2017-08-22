#ifndef PASSTHROUGHFILTERCPP_H
#define PASSTHROUGHFILTERCPP_H
#include <iostream>
#include <string>

class PassThroughFiltercpp
{
public:
    PassThroughFiltercpp();
    void ApplyFilter(std::string fileName);
    void SearchCircle(std::string fileName, float radius);
    void SearchVoxel(std::string fileNam);
    void SearchKNearest(std::string fileName, int K);
};

#endif // PASSTHROUGHFILTERCPP_H
