#pragma once

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>>
#include <vector>
#include <string>
#include <limits.h>


class CMapConstruct
{
public:
    CMapConstruct()
    {
        m_dMapInterval = 1.0;
        m_nLatestCapacity = 20;
    }

    int AddFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& points,
                 Eigen::Isometry3d& pose);
    int GetMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& points);

private:
    double m_dMapInterval;
    unsigned int m_nLatestCapacity;
    pcl::PointCloud<pcl::PointXYZI> m_PtList;
    std::vector<Eigen::Isometry3d> m_poseList;
    std::vector<size_t> m_PtContList;
};
