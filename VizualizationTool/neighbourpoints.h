#ifndef NEIGHBORPOINTS_H
#define NEIGHBORPOINTS_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

class NeighbourPoints
{
private:
    pcl::PointXYZ _searchedPoint;
    std::vector<pcl::PointXYZ> _neighbourCloud;
public:
    NeighbourPoints(pcl::PointXYZ searchedPoint, std::vector<pcl::PointXYZ> neighbourCloud);
    std::vector<pcl::PointXYZ> GetNeighbourCloud();
};

#endif // NEIGHBORPOINTS_H
