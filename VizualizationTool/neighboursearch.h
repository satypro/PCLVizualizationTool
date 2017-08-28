#ifndef NEIGHBOURSEARCH_H
#define NEIGHBOURSEARCH_H
#include <string>
#include "neighbourpoints.h"


class NeighbourSearch
{
public:
    NeighbourSearch();
    NeighbourPoints* GetRadiusSearchNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& searchPoint, float radius);
};

#endif // NEIGHBOURSEARCH_H
