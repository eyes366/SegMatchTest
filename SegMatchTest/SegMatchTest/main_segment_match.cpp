#include <iostream>
#include <unistd.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/timer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations


#include <pcl/io/pcd_io.h>
#include "hdl_grabber_.h"
#include "vlp_grabber_.h"
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <typeinfo>

#include <pcl/features/normal_3d.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "SegmentRegionGrow.h"
#include "OpenCVInc.h"
#include "SegmentDefine.h"

using namespace std;
using namespace cv;
using namespace pcl;

main (int argc, char *argv[])
{
    vector<Mat> features_list;
    vector<int> index_list;
    for (size_t i = 0u; i < 2000; i++)
    {
        char szLog[1024] = {0};
        sprintf(szLog, "ObjectFeature%06d.yml", i);
        FileStorage fs(szLog, FileStorage::READ);
        Mat features;
        fs["features"] >> features;
        if (features.rows > 0)
        {
            features_list.push_back(features);
            index_list.push_back(i);
            cout << i << ":" << features.rows << endl;
        }
    }
    cout << "features_list.size() " << features_list.size() << endl;

    int test0_ind = atoi(argv[1])/*25*/;
    int test1_ind = atoi(argv[2])/*30*/;
    cout << "test0_ind: " << test0_ind << "   test1_ind: " << test1_ind << endl;
    Mat test0 = features_list[test0_ind];
    Mat test1 = features_list[test1_ind];

    CvRTrees rtrees_;
    rtrees_.load("/home/xinzi/catkin_ws/src/segmatch/laser_mapper/demonstration_files/kitti/random_forest_eigen_25trees.xml");
    Mat confidence = compute_confidence_cross(test0, test1, rtrees_);//0.7 || 0.65
    confidence.setTo(Scalar(0), confidence<0.7);
    ofstream fs;
    fs.open("confidence.csv", ios_base::out);
    for (unsigned int i = 0; i < confidence.rows; i++)
    {
        for (unsigned int j = 0; j < confidence.cols; j++)
        {
            fs << confidence.at<float>(i,j) << ",";
        }
        fs << "\n";
    }
    fs.close();

    vector<segmatch::Segment> cluster0;
//    vector<PointCloud<PointXYZI>::Ptr> test0_pcd_list;
    for (unsigned int i = 0; i < test0.rows; i++)
    {
        PointCloud<PointXYZI>::Ptr temp(new PointCloud<PointXYZI>);
        char szPCD[1024] = {0};
        sprintf(szPCD, "ObjectPoints%06d_%06d.pcd", index_list[test0_ind], i);
        PCDReader Reader;
        Reader.read(szPCD, *temp);
//        test0_pcd_list.push_back(temp);
        segmatch::Segment seg;
        seg.point_cloud = (*temp);
        seg.frame = index_list[test0_ind];
        seg.segment_id = i;
        seg.feature = features_list[test0_ind].row(i);
        seg.calculateCentroid();
        cluster0.push_back(seg);
    }
    cout << "cluster0.size() " << cluster0.size() << endl;

    vector<segmatch::Segment> cluster1;
//    vector<PointCloud<PointXYZI>::Ptr> test1_pcd_list;
    for (unsigned int i = 0; i < test1.rows; i++)
    {
        PointCloud<PointXYZI>::Ptr temp(new PointCloud<PointXYZI>);
        char szPCD[1024] = {0};
        sprintf(szPCD, "ObjectPoints%06d_%06d.pcd", index_list[test1_ind], i);
        PCDReader Reader;
        Reader.read(szPCD, *temp);
//        test1_pcd_list.push_back(temp);
        segmatch::Segment seg;
        seg.point_cloud = (*temp);
        seg.frame = index_list[test1_ind];
        seg.segment_id = i;
        seg.feature = features_list[test1_ind].row(i);
        seg.calculateCentroid();
        cluster1.push_back(seg);
    }
    cout << "cluster1.size() " << cluster1.size() << endl;

    vector<segmatch::Match> matches;
    match_clusters(cluster0, cluster1, matches, rtrees_);
    cout << "matches.size()" << matches.size() << endl;

    vector<int> valid_indicate;
    filter_match(cluster0, cluster1, matches, valid_indicate);

    display_cluster_and_matches(cluster0, cluster1, matches, &valid_indicate);



    return 0;
}
