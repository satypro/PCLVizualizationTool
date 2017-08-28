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
#include "passthroughfiltercpp.h"
#include "neighbourpoints.h"
#include "neighboursearch.h"

PassThroughFiltercpp::PassThroughFiltercpp()
{

}

void PassThroughFiltercpp::SearchCircle(std::string fileName, float radius )
{
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

      if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file some_pcd.pcd \n");
        return;
      }

      pcl::PointXYZ searchPoint = cloud->points[232459];
      NeighbourSearch* n =  new NeighbourSearch();
      NeighbourPoints* neighbour = n->GetRadiusSearchNeighbour(cloud, searchPoint, radius);
      
      std::vector<pcl::PointXYZ> points = neighbour->GetNeighbourCloud();
      for (size_t i = 0; i < points.size (); ++i)
      {
            std::cout << "    " << points[i].x
                      << " "    << points[i].y
                      << " "    << points[i].z << std::endl;

      }
      /*std::cout << "Loaded "
                << cloud->width * cloud->height
                << " data points from some_pcd.pcd with the following fields: "
                << std::endl;
      for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;
      */

      /*
      float resolution = 128.0f;
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

      octree.setInputCloud (cloud);
      octree.addPointsFromInputCloud ();

      for (size_t i = 0; i < cloud->points.size (); ++i)
      {
          pcl::PointXYZ searchPoint = cloud->points[i];

          // Neighbors within radius search

          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;

          std::cout << i <<" : "<<"   ---> "<< "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;

          if (isnan(searchPoint.x))
              continue;

          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

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

              cloud_cluster->points.push_back(cloud->points[ pointIdxRadiusSearch[i] ]);
            }
          }

          pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
          viewer.showCloud(cloud_cluster);
          while (!viewer.wasStopped ())
          {

          }
      }
      */
}

void PassThroughFiltercpp::SearchVoxel(std::string fileName)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file some_pcd.pcd \n");
      return;
    }

    /*std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from some_pcd.pcd with the following fields: "
              << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
      std::cout << "    " << cloud->points[i].x
                << " "    << cloud->points[i].y
                << " "    << cloud->points[i].z << std::endl;
    */

    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        pcl::PointXYZ searchPoint = cloud->points[i];

         // Neighbors within voxel search

        std::cout << i <<" : "<<"   ---> "<< "voxel search (" << searchPoint.x
            << " " << searchPoint.y
            << " " << searchPoint.z
            << std::endl;

        if (isnan(searchPoint.x))
            continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> pointIdxVec;

        if (octree.voxelSearch (searchPoint, pointIdxVec))
        {
            std::cout << "Neighbors within voxel search at (" << searchPoint.x
             << " " << searchPoint.y
             << " " << searchPoint.z << ")"
             << std::endl;

            for (size_t i = 0; i < pointIdxVec.size (); ++i)
            {
               std::cout << "    " << cloud->points[pointIdxVec[i]].x
                    << " " << cloud->points[pointIdxVec[i]].y
                    << " " << cloud->points[pointIdxVec[i]].z << std::endl;

                cloud_cluster->points.push_back(cloud->points[pointIdxVec[i]]);
            }
        }

        pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        viewer.showCloud(cloud_cluster);
        while (!viewer.wasStopped ())
        {

        }
    }
}

void PassThroughFiltercpp::SearchKNearest(std::string fileName, int k)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file some_pcd.pcd \n");
      return;
    }

    /*std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from some_pcd.pcd with the following fields: "
              << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
      std::cout << "    " << cloud->points[i].x
                << " "    << cloud->points[i].y
                << " "    << cloud->points[i].z << std::endl;
    */

    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        pcl::PointXYZ searchPoint = cloud->points[i];

         // K nearest neighbor search

        std::cout << "K nearest neighbor search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z
                  << ") with K=" << k << std::endl;

        if (isnan(searchPoint.x))
            continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;

        if (octree.nearestKSearch (searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
              std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
                        << " " << cloud->points[ pointIdxNKNSearch[i] ].y
                        << " " << cloud->points[ pointIdxNKNSearch[i] ].z
                        << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;

              cloud_cluster->points.push_back(cloud->points[pointIdxNKNSearch[i]]);
            }
         }

        pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        viewer.showCloud(cloud_cluster);
        while (!viewer.wasStopped ())
        {

        }
    }
}

void PassThroughFiltercpp::ApplyFilter(std::string fileName)
{
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

     if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
     {
       PCL_ERROR ("Couldn't read file some_pcd.pcd \n");
       return;
     }

     // Create the filtering object
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud (cloud);
     pass.setFilterFieldName ("z");
     pass.setFilterLimits (0.0, 1.0);
     //pass.setFilterLimitsNegative (true);
     pass.filter (*cloud_filtered);

     std::cerr << "Cloud after filtering: " << std::endl;
     for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
       std::cerr << "    " << cloud_filtered->points[i].x << " "
                           << cloud_filtered->points[i].y << " "
                           << cloud_filtered->points[i].z << std::endl;

     pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
     viewer.showCloud(cloud_filtered);
     while (!viewer.wasStopped ())
     {

     }
}

