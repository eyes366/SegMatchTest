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

#include <math.h>

#include "SegmentRegionGrow.h"


using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

#include "OpenCVInc.h"

#include <math.h>

#include "VLP16Graber.h"
#include "HDL32Grab.h"
#include "TrjLogReader.h"
#include "MapConstruct.h"

void GetGroundPoints(PointCloud<PointXYZI>::Ptr pts_in,
    PointCloud<PointXYZI>::Ptr pts_out,
    PointCloud<PointXYZI>::Ptr pts_obj_out,
    pcl::ModelCoefficients::Ptr coefficients);

bool g_bIsPause = false;

void visualization_button_callback (const pcl::visualization::KeyboardEvent &event,
                                    void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
      if ((event.getKeySym () == "s" || event.getKeySym () == "S")&& event.keyDown ())
      {
            g_bIsPause = !g_bIsPause;
      }
}

double g_SlopeThreshHold = 0.0;

double g_dHeading = 0.0;
double g_dRoll = 0.0;
double g_dPitch = 0.0;

int main(int argc, char ** argv)
{
    CTrjLogReader TrjReader;
    TrjReader.ReadLog("/home/xinzi/data/src/20180110172343.txt");
    CMapConstruct MapOpr;

    int nSourceType = atoi(argv[1]);
    char* pszSourceAddr = argv[2];
    g_SlopeThreshHold = atof(argv[3]);
    g_dHeading = atof(argv[4]);
    g_dRoll = atof(argv[5]);
    g_dPitch = atof(argv[6]);

    Eigen::Affine3f pose =
            Eigen::Translation3f (Eigen::Vector3f (0, 0.0, 0)) *
            Eigen::AngleAxisf (g_dHeading/180.0*CV_PI,   Eigen::Vector3f::UnitZ ()) *
            Eigen::AngleAxisf (g_dRoll/180.0*CV_PI, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (g_dPitch/180.0*CV_PI,  Eigen::Vector3f::UnitX ());

    VLPGrabber* pGrabber = NULL;
    if (nSourceType == 0)
    {
        pGrabber = new VLPGrabber(boost::asio::ip::address_v4::from_string(pszSourceAddr),2368);
    }
    else
    {
        pGrabber = new VLPGrabber(pszSourceAddr);
    }

    SimpleVLPViewer<PointXYZI> v(*pGrabber);

    v.setMode(1);
    v.start();

    pcl::visualization::PCLVisualizer* p = NULL;
    int vp_1, vp_2;
    p = new pcl::visualization::PCLVisualizer(argc,argv,"Pairwise Incremental Registration example");
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->setCameraPosition(0.0, 0.0, 20, 0.0, 0.0, 0.0, vp_1);
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    p->setCameraPosition(0.0, 0.0, 20, 0.0, 0.0, 0.0, vp_2);
    p->addCoordinateSystem(1.0,"vp_1" , vp_1);
    p->addCoordinateSystem(1.0,"vp_2" , vp_2);

    boost::signals2::connection button_connection = p->registerKeyboardCallback(visualization_button_callback, (void*)p);

    int nCont = 0;
    while(1)
    {
        PointCloud<PointXYZI>::ConstPtr ptCloudPt;
        int nRt = v.nextFrame(ptCloudPt);
        if (nRt == 0)
        {
            cout << "Get pcap data failed!" << endl;
            p->spin();
            continue;
        }
        nCont++;
        boost::timer ter;

        Eigen::Isometry3d pose_global;
        if (TrjReader.GetPoseByTime(ptCloudPt->header.stamp, pose_global) < 0)
        {
            cout << "TrjReader.GetPoseByTime   failed!+++++++++++++++++++++++" << endl;
            continue;
        }

        std::cout << "Point cont:" << ptCloudPt->size() << std::endl;

        PointCloud<PointXYZI>::Ptr CloudOri_rot(new PointCloud<PointXYZI>);
        pcl::transformPointCloud (*ptCloudPt, *CloudOri_rot, pose);

        float dHeightAbove = -0.3;
        float dHeightBelow = 3.0;
        PointCloud<PointXYZI>::Ptr CloudOri(new PointCloud<PointXYZI>);
        CloudOri->reserve(CloudOri_rot->size());
        for(int i = 0; i < CloudOri_rot->size(); i++)
        {
            PointXYZI pt = CloudOri_rot->at(i);
            if(boost::math::isnan(pt.x) ||
                    boost::math::isnan(pt.y) ||
                    boost::math::isnan(pt.z))
            {
                continue;
            }
            if (pt.z <= dHeightAbove || pt.z >= dHeightBelow)
            {
                continue;
            }
            double dDist = sqrt(pow(pt.x,2) + pow(pt.y,2));
//            if (dDist >= 50.0)
//            {
//                continue;
//            }
            CloudOri->push_back(pt);
        }

        pcl::transformPointCloud (*CloudOri, *CloudOri, pose.inverse());
        int nAddRt = MapOpr.AddFrame(CloudOri, pose_global);
        if (nAddRt > 0)
        {
            PointCloud<PointXYZI>::Ptr CloudMap(new PointCloud<PointXYZI>);
            PointCloud<PointXYZRGB>::Ptr CloudColor(new PointCloud<PointXYZRGB>);
            MapOpr.GetMap(CloudMap);
            segment_process(nCont, CloudMap, CloudColor);

            PointCloudColorHandler<pcl::PointXYZRGB>* handler_2 = new PointCloudColorHandlerRGBField<PointXYZRGB> (CloudColor);
            p->removePointCloud("2",vp_2);
            p->addPointCloud(CloudColor,*handler_2,"2",vp_2);
            delete handler_2;
        }

        /*

        PointCloud<PointXYZI>::Ptr CloudDown(new PointCloud<PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud (CloudOri);
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
        sor.filter (*CloudDown);
        (*CloudOri) = (*CloudDown);
        std::cout << "Down sample point cont: " << CloudOri->size() << std::endl;

        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_;
        sor_.setInputCloud (CloudOri);
        sor_.setMeanK (30);
        sor_.setStddevMulThresh (0.5);
        PointCloud<PointXYZI>::Ptr cloud_filtered(new PointCloud<PointXYZI>);
        sor_.filter (*cloud_filtered);
        (*CloudOri) = (*cloud_filtered);

        std::vector<pcl::PointIndices> clusters;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new PointCloud<PointXYZRGB>);
//        region_growing_segmentation_process(CloudOri, clusters);
        euclidean_segmentation_process(CloudOri, clusters);
        color_segmentation(CloudOri, clusters, colored_cloud);
        std::cout << "Segment cont: " << clusters.size() << std::endl;

        vector<PointCloud<ESFSignature640>::Ptr> vec_signature;
        ensemble_shape_functions_process(CloudOri, clusters, vec_signature);

        char szFilePath[1024] = {0};
        sprintf(szFilePath, "signatures%06d.csv", nCont);
        ofstream fs(szFilePath);
        for (unsigned int i = 0; i < vec_signature.size(); i++)
        {
            for (unsigned int j = 0; j < 640; j++)
            {
                fs << vec_signature[i]->points[0].histogram[j] << ",";
            }
            fs << endl;
        }
        */

        PointCloudColorHandler<pcl::PointXYZI>* handler_1 = new PointCloudColorHandlerGenericField<PointXYZI> (CloudOri,"intensity");
        p->removePointCloud("1",vp_1);
        p->addPointCloud(CloudOri,*handler_1, "1", vp_1);
        delete handler_1;

//        pcl::PointCloud<PointXYZI>::Ptr pMap(new pcl::PointCloud<PointXYZI>);
//        MapOpr.GetMap(pMap);
//        PointCloudColorHandler<pcl::PointXYZI>* handler_3 = new PointCloudColorHandlerGenericField<PointXYZI> (pMap,"intensity");
//        p->removePointCloud("3",vp_2);
//        p->addPointCloud(pMap,*handler_3, "3", vp_2);
//        delete handler_3;

//        PointCloudColorHandler<pcl::PointXYZRGB>* handler_2 = new PointCloudColorHandlerRGBField<PointXYZRGB> (colored_cloud);
//        p->removePointCloud("2",vp_2);
//        p->addPointCloud(colored_cloud,*handler_2,"2",vp_2);
//        delete handler_2;

        std::cout<< "Time cost: " <<ter.elapsed()<<std::endl;

//        g_bIsPause = true;
        while(g_bIsPause)
        {
            p->spinOnce();
        }

        p->spinOnce();


    }

    button_connection.disconnect();
    return (0);
}


