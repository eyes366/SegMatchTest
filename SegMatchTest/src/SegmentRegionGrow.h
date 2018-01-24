#ifndef _SEGMENT_REGION_GROW_H_
#define _SEGMENT_REGION_GROW_H_

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <time.h>

#include <pcl/features/esf.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "OpenCVInc.h"
#include "SegmentDefine.h"


int segment_process(int nInd, pcl::PointCloud<pcl::PointXYZI>::Ptr& Cloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& seg_clolor);

int color_segmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       std::vector<pcl::PointIndices>& clusters,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud);

int euclidean_segmentation_process(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                        std::vector<pcl::PointIndices>& clusters);

int region_growing_segmentation_process(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                        std::vector<pcl::PointIndices>& clusters);

int ensemble_shape_functions_process(pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
                                     std::vector<pcl::PointIndices>& clusters_in,
                                     std::vector<pcl::PointCloud<pcl::ESFSignature640>::Ptr>& vec_signature);

cv::Vec<double,7> eigen_value_base_extract(pcl::PointCloud<pcl::PointXYZI>::Ptr points_in);

cv::Mat compute_eigen_features_distance(cv::Mat& feature1, cv::Mat& feature2);
//
cv::Mat compute_confidence_cross(cv::Mat& feature1, cv::Mat& feature2, CvRTrees& tree);

cv::Mat eigen_value_base_extract_batch(int nFrame, pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
                                 std::vector<pcl::PointIndices>& clusters_in, bool bIsSavePt = false);

int match_clusters(std::vector<segmatch::Segment>& cluster1, std::vector<segmatch::Segment>& cluster2,
                   std::vector<segmatch::Match>& matches, CvRTrees& tree, float confidence = 0.7);

int filter_match(std::vector<segmatch::Segment>& cluster1, std::vector<segmatch::Segment>& cluster2,
                 std::vector<segmatch::Match>& matches, std::vector<int>& valid_mask);

void display_cluster_and_matches(std::vector<segmatch::Segment>& cluster1, std::vector<segmatch::Segment>& cluster2,
                                 std::vector<segmatch::Match>& matches, std::vector<int>* pvalid_mask = 0);

#endif
