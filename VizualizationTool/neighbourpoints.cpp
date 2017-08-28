#include "neighbourpoints.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

NeighbourPoints::NeighbourPoints(pcl::PointXYZ searchedPoint, std::vector<pcl::PointXYZ> neighbourCloud)
{
    _searchedPoint = searchedPoint;
    _neighbourCloud = neighbourCloud;
}

std::vector<pcl::PointXYZ> NeighbourPoints::GetNeighbourCloud()
{
    return _neighbourCloud;
}
