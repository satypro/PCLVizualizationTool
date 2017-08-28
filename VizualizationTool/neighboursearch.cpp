#include "neighboursearch.h"
#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

NeighbourSearch::NeighbourSearch()
{

}

NeighbourPoints* NeighbourSearch::GetRadiusSearchNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& searchPoint , float radius)
{
    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();

        // Neighbors within radius search

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        std::cout << 500 <<" : "<<"   ---> "<< "Neighbors within radius search at (" << searchPoint.x
            << " " << searchPoint.y
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;

        if (isnan(searchPoint.x))
            return NULL;

        std::vector<pcl::PointXYZ> _neighbourCloud;
        if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
              for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
              {
                std::cout << "    " <<searchPoint.x
                          << "    " <<searchPoint.y
                          << "    " <<searchPoint.z
                          << "-->" << cloud->points[ pointIdxRadiusSearch[i] ].x
                          << "   " << cloud->points[ pointIdxRadiusSearch[i] ].y
                          << "   " << cloud->points[ pointIdxRadiusSearch[i] ].z
                          << "   (squared distance: " << pointRadiusSquaredDistance[i] << ")"
                          << std::endl;

                _neighbourCloud.push_back(cloud->points[ pointIdxRadiusSearch[i] ]);
              }
        }

        NeighbourPoints* neighbour = new NeighbourPoints(searchPoint, _neighbourCloud);

        return neighbour;
}
