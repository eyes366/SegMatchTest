#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>//icp_nl.h
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include "OpenCVInc.h"

namespace segmatch
{

class Match
{
public:
    int id1;
    int id2;
    pcl::PointXYZ center1;
    pcl::PointXYZ center2;
    cv::Mat feature1;
    cv::Mat feature2;
    float confidence;
};

class Segment
{
public:
    Segment(){}
    void calculateCentroid()
    {
        int kNPoints = point_cloud.size();
        pcl::PointXYZ ptCenter(0.0, 0.0, 0.0);
        for (size_t i = 0u; i < kNPoints; ++i)
        {
            ptCenter.x += point_cloud[i].x;
            ptCenter.y += point_cloud[i].y;
            ptCenter.z += point_cloud[i].z;
        }
        ptCenter.x /= double(kNPoints);
        ptCenter.y /= double(kNPoints);
        ptCenter.z /= double(kNPoints);
        center = ptCenter;
    }

    int segment_id;
    int frame;
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    cv::Mat feature;
    pcl::PointXYZ center;
    uint64_t time_stamp;
};



}
