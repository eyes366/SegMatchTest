#include "SegmentRegionGrow.h"

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
#include <pcl/recognition/cg/geometric_consistency.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

using namespace pcl;
using namespace std;
using namespace cv;

int segment_process(int nInd, PointCloud<PointXYZI>::Ptr& Cloud,
                    PointCloud<PointXYZRGB>::Ptr& seg_clolor)
{
    PointCloud<PointXYZI>::Ptr CloudDown(new PointCloud<PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (Cloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*CloudDown);
//    (*CloudDown) = (*Cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_;
    sor_.setInputCloud (CloudDown);
    sor_.setMeanK (30);
    sor_.setStddevMulThresh (0.5);
    PointCloud<PointXYZI>::Ptr cloud_filtered(new PointCloud<PointXYZI>);
    sor_.filter (*cloud_filtered);

    std::vector<pcl::PointIndices> clusters;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new PointCloud<PointXYZRGB>);
//        region_growing_segmentation_process(CloudOri, clusters);
    euclidean_segmentation_process(cloud_filtered, clusters);
    color_segmentation(cloud_filtered, clusters, seg_clolor);
    std::cout << "Segment cont: " << clusters.size() << std::endl;

    eigen_value_base_extract_batch(nInd, cloud_filtered, clusters, true);

//    vector<PointCloud<ESFSignature640>::Ptr> vec_signature;
//    ensemble_shape_functions_process(cloud_filtered, clusters, vec_signature);

//    char szFilePath[1024] = {0};
//    sprintf(szFilePath, "signatures%06d.csv", nInd);
//    ofstream fs(szFilePath);
//    for (unsigned int i = 0; i < vec_signature.size(); i++)
//    {
//        for (unsigned int j = 0; j < 640; j++)
//        {
//            fs << vec_signature[i]->points[0].histogram[j] << ",";
//        }
//        fs << endl;
//    }

}

int color_segmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       std::vector<pcl::PointIndices>& clusters,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>);

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters.size (); i_segment++)
    {
        colors.push_back (static_cast<unsigned char> (rand () % 256));
        colors.push_back (static_cast<unsigned char> (rand () % 256));
        colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud_->width = cloud->width;
    colored_cloud_->height = cloud->height;
    colored_cloud_->is_dense = cloud->is_dense;
    for (size_t i_point = 0; i_point < cloud->size (); i_point++)
    {
        pcl::PointXYZRGB point;
        point.x = *(cloud->points[i_point].data);
        point.y = *(cloud->points[i_point].data + 1);
        point.z = *(cloud->points[i_point].data + 2);
        point.r = 0;
        point.g = 0;
        point.b = 0;
        colored_cloud_->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters.begin (); i_segment != clusters.end (); i_segment++)
    {
        std::vector<int>::iterator i_point;
        for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
        {
            int index;
            index = *i_point;
            pcl::PointXYZRGB pt;
            colored_cloud_->points[index].r = colors[3 * next_color];
            colored_cloud_->points[index].g = colors[3 * next_color + 1];
            colored_cloud_->points[index].b = colors[3 * next_color + 2];
        }
        next_color++;
    }

    (*colored_cloud) = (*colored_cloud_);

    return clusters.size();
}

int euclidean_segmentation_process(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                        std::vector<pcl::PointIndices>& clusters)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::VoxelGrid<pcl::PointXYZI> vg;
//    vg.setInputCloud (cloud);
//    vg.setLeafSize (0.1f, 0.1f, 0.1f);
//    vg.filter (*cloud_filtered);
    (*cloud_filtered) = (*cloud);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud_filtered);

//    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.2); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (clusters);

    return clusters.size();
}

int region_growing_segmentation_process(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                        std::vector<pcl::PointIndices>& clusters)
{
    clusters.clear();
//    colored_cloud->clear();
    if (cloud->size() <= 0)
    {
        return 0;
    }
    pcl::search::Search<pcl::PointXYZI>::Ptr tree =
            boost::shared_ptr<pcl::search::Search<pcl::PointXYZI> > (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
//    normal_estimator.setKSearch (50);
    normal_estimator.setRadiusSearch(0.5);
    normal_estimator.compute (*normals);

    //    pcl::IndicesPtr indices (new std::vector <int>);
    //    pcl::PassThrough<pcl::PointXYZI> pass;
    //    pass.setInputCloud (cloud);
    //    pass.setFilterFieldName ("z");
    //    pass.setFilterLimits (0.0, 1.0);
    //    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize (20);
    reg.setMaxClusterSize (50000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (/*8.0*/3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (/*5.0*/1.0);

    reg.extract (clusters);

    return clusters.size();

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_temp = reg.getColoredCloud ();
//    (*colored_cloud) = (*pt_temp);

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>);

//    srand (static_cast<unsigned int> (time (0)));
//    std::vector<unsigned char> colors;
//    for (size_t i_segment = 0; i_segment < clusters.size (); i_segment++)
//    {
//        colors.push_back (static_cast<unsigned char> (rand () % 256));
//        colors.push_back (static_cast<unsigned char> (rand () % 256));
//        colors.push_back (static_cast<unsigned char> (rand () % 256));
//    }

//    colored_cloud_->width = cloud->width;
//    colored_cloud_->height = cloud->height;
//    colored_cloud_->is_dense = cloud->is_dense;
//    for (size_t i_point = 0; i_point < cloud->size (); i_point++)
//    {
//        pcl::PointXYZRGB point;
//        point.x = *(cloud->points[i_point].data);
//        point.y = *(cloud->points[i_point].data + 1);
//        point.z = *(cloud->points[i_point].data + 2);
//        point.r = 0;
//        point.g = 0;
//        point.b = 0;
//        colored_cloud_->points.push_back (point);
//    }

//    std::vector< pcl::PointIndices >::iterator i_segment;
//    int next_color = 0;
//    for (i_segment = clusters.begin (); i_segment != clusters.end (); i_segment++)
//    {
//        std::vector<int>::iterator i_point;
//        for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
//        {
//            int index;
//            index = *i_point;
//            pcl::PointXYZRGB pt;
//            colored_cloud_->points[index].r = colors[3 * next_color];
//            colored_cloud_->points[index].g = colors[3 * next_color + 1];
//            colored_cloud_->points[index].b = colors[3 * next_color + 2];
//        }
//        next_color++;
//    }

//    (*colored_cloud) = (*colored_cloud_);

//    return clusters.size();
}

int ensemble_shape_functions_process(pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
                                     std::vector<pcl::PointIndices>& clusters_in,
                                     std::vector<pcl::PointCloud<pcl::ESFSignature640>::Ptr>& vec_signature)
{
    vec_signature.resize(clusters_in.size());
    for (unsigned int i = 0; i < vec_signature.size(); i++)
    {
        pcl::PointCloud<pcl::ESFSignature640>::Ptr temp(new pcl::PointCloud<pcl::ESFSignature640>);
        vec_signature[i] = temp;
    }

    for (unsigned int i = 0; i < clusters_in.size(); i++)
    {
//        pcl::PointCloud<pcl::ESFSignature640>::Ptr signatureT(new pcl::PointCloud<pcl::ESFSignature640>);
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(points_in);
        pcl::PointIndices::Ptr ppp_( new pcl::PointIndices );
        (*ppp_) = clusters_in[i];
        extract.setIndices(ppp_);
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr points_target( new pcl::PointCloud<pcl::PointXYZI>);
        extract.filter(*points_target);

        pcl::ESFEstimation<pcl::PointXYZI, pcl::ESFSignature640> esf_estimator_;
        esf_estimator_.setInputCloud(points_target);
//        pcl::PointIndices::Ptr ppp( &(clusters_in[i]) );


//        esf_estimator_.setIndices(ppp_);
        esf_estimator_.compute(*(vec_signature[i]));
//        vec_signature.push_back(signatureT);
//        for (unsigned int j = 0; j < 10; j++)
//        {
//            std::cout << vec_signature[i]->points[0].histogram[j] << std::endl;
//        }
//        std::cout << std::endl;
    }

    return 0;
}

template<typename T>
bool swap_if_gt(T& a, T& b) {
  if (a > b) {
    std::swap(a, b);
    return true;
  }
  return false;
}

cv::Vec<double,7> eigen_value_base_extract(pcl::PointCloud<pcl::PointXYZI>::Ptr points_in)
{
    cv::Vec<double,7> feature;
    const size_t kNPoints = points_in->size();
    if (kNPoints <= 0)
    {
        cout << "kNPoints <= 0" << endl;
        getchar();
    }

    //Calculate the center of point cloud
    PointXYZ ptCenter(0.0, 0.0, 0.0);
    for (size_t i = 0u; i < kNPoints; ++i)
    {
        ptCenter.x += points_in->at(i).x;
        ptCenter.y += points_in->at(i).y;
        ptCenter.z += points_in->at(i).z;
    }
    ptCenter.x /= double(kNPoints);
    ptCenter.y /= double(kNPoints);
    ptCenter.z /= double(kNPoints);

    // Find the variances.
    PointCloud<pcl::PointXYZ> variances;
    for (size_t i = 0u; i < kNPoints; ++i)
    {
        variances.push_back(pcl::PointXYZ());
        variances[i].x = points_in->at(i).x - ptCenter.x;
        variances[i].y = points_in->at(i).y - ptCenter.y;
        variances[i].z = points_in->at(i).z - ptCenter.z;
    }

    // Find the covariance matrix. Since it is symmetric, we only bother with the upper diagonal.
    const std::vector<size_t> row_indices_to_access = {0,0,0,1,1,2};
    const std::vector<size_t> col_indices_to_access = {0,1,2,1,2,2};
    Eigen::Matrix3f covariance_matrix;
    for (size_t i = 0u; i < row_indices_to_access.size(); ++i)
    {
        const size_t row = row_indices_to_access[i];
        const size_t col = col_indices_to_access[i];
        double covariance = 0;
        for (size_t k = 0u; k < kNPoints; ++k)
        {
            covariance += variances.points[k].data[row] * variances.points[k].data[col];
        }
        covariance /= kNPoints;
        covariance_matrix(row,col) = covariance;
        covariance_matrix(col,row) = covariance;
    }

    // Compute eigenvalues of covariance matrix.
    constexpr bool compute_eigenvectors = false;
    Eigen::EigenSolver<Eigen::Matrix3f> eigenvalues_solver(covariance_matrix, compute_eigenvectors);
    std::vector<float> eigenvalues(3, 0.0);
    eigenvalues.at(0) = eigenvalues_solver.eigenvalues()[0].real();
    eigenvalues.at(1) = eigenvalues_solver.eigenvalues()[1].real();
    eigenvalues.at(2) = eigenvalues_solver.eigenvalues()[2].real();
    if (eigenvalues_solver.eigenvalues()[0].imag() != 0.0 ||
            eigenvalues_solver.eigenvalues()[1].imag() != 0.0 ||
            eigenvalues_solver.eigenvalues()[2].imag() != 0.0 )
    {
        cout << "eigenvalues_solver.eigenvalues()[0].imag() != 0.0" << endl;
        getchar();
    }

    // Sort eigenvalues from smallest to largest.
    swap_if_gt(eigenvalues.at(0), eigenvalues.at(1));
    swap_if_gt(eigenvalues.at(0), eigenvalues.at(2));
    swap_if_gt(eigenvalues.at(1), eigenvalues.at(2));

    // Normalize eigenvalues.
    double sum_eigenvalues = eigenvalues.at(0) + eigenvalues.at(1) + eigenvalues.at(2);
    double e1 = eigenvalues.at(0) / sum_eigenvalues;
    double e2 = eigenvalues.at(1) / sum_eigenvalues;
    double e3 = eigenvalues.at(2) / sum_eigenvalues;
    if (e1 == e2 || e2 == e3 || e1 == e3)
    {
        cout << "e1 == e2 || e2 == e3 || e1 == e3" << endl;
        getchar();
    }

    // Store inside features.
    const double sum_of_eigenvalues = e1 + e2 + e3;
    constexpr double kOneThird = 1.0/3.0;
    if (e1 == 0.0 || sum_of_eigenvalues == 0.0)
    {
        cout << "e1 == 0.0 || sum_of_eigenvalues == 0.0" << endl;
        getchar();
    }

    const double kNormalizationPercentile = 1.0;

    const double kLinearityMax = 28890.9 * kNormalizationPercentile;
    const double kPlanarityMax = 95919.2 * kNormalizationPercentile;
    const double kScatteringMax = 124811 * kNormalizationPercentile;
    const double kOmnivarianceMax = 0.278636 * kNormalizationPercentile;
    const double kAnisotropyMax = 124810 * kNormalizationPercentile;
    const double kEigenEntropyMax = 0.956129 * kNormalizationPercentile;
    const double kChangeOfCurvatureMax = 0.99702 * kNormalizationPercentile;

    const double kNPointsMax = 13200 * kNormalizationPercentile;

    feature[0] = (e1 - e2) / e1 / kLinearityMax;
    feature[1] = (e2 - e3) / e1 / kPlanarityMax;
    feature[2] = e3 / e1 / kScatteringMax;
    feature[3] = std::pow(e1 * e2 * e3, kOneThird) / kOmnivarianceMax;
    feature[4] = (e1 - e3) / e1 / kAnisotropyMax;
    feature[5] = (e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)) / kEigenEntropyMax;
    feature[6] = e3 / sum_of_eigenvalues / kChangeOfCurvatureMax;

    return feature;
}

cv::Mat compute_confidence_cross(cv::Mat& feature1, cv::Mat& feature2, CvRTrees& tree)
{
    Mat out = Mat::zeros(feature1.rows, feature2.rows, CV_32F);
    for (unsigned int i = 0; i < feature1.rows; i++)
    {
        for (unsigned int j = 0; j < feature2.rows; j++)
        {
            Mat feature1_row = feature1.row(i);
            Mat feature2_row = feature2.row(j);
            Mat feature_diff = compute_eigen_features_distance(
                        feature1_row, feature2_row);
            Mat feature_diff_;
            feature_diff.convertTo(feature_diff_, CV_32F);
            float confidence = tree.predict_prob(feature_diff_);
            out.at<float>(i,j) = confidence;
        }
    }

    return out;
}

cv::Mat compute_eigen_features_distance(cv::Mat& feature1, cv::Mat& feature2)
{
    Mat diff_out = Mat::zeros(1, 35, CV_64F);
    Mat f_diff = cv::abs(feature1 - feature2);
    f_diff.copyTo(diff_out(Range::all(),Range(0,7)));

    Mat f1_abs = cv::abs(feature1);
    Mat f2_abs = cv::abs(feature2);

    Mat f_diff_norm_2 = f_diff/f2_abs;
    Mat f_diff_norm_1 = f_diff/f1_abs;

    f_diff_norm_2.copyTo(diff_out(Range::all(),Range(7,14)));
    f_diff_norm_1.copyTo(diff_out(Range::all(),Range(14,21)));
    f1_abs.copyTo(diff_out(Range::all(),Range(21,28)));
    f1_abs.copyTo(diff_out(Range::all(),Range(28,35)));

    return diff_out;
}

cv::Mat eigen_value_base_extract_batch(int nFrame, pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
                                       std::vector<pcl::PointIndices>& clusters_in,
                                       bool bIsSavePt)
{
    vector<cv::Vec<double,7> > line_numbers;

    for (unsigned int i = 0; i < clusters_in.size(); i++)
    {
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(points_in);
        pcl::PointIndices::Ptr ppp_( new pcl::PointIndices );
        (*ppp_) = clusters_in[i];
        extract.setIndices(ppp_);
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr points_target( new pcl::PointCloud<pcl::PointXYZI>);
        extract.filter(*points_target);
        cv::Vec<float,7> feature = eigen_value_base_extract(points_target);
        line_numbers.push_back(feature);
        if (bIsSavePt)
        {
            char szLog[1024] = {0};
            sprintf(szLog, "ObjectPoints%06d_%06d.pcd", nFrame, i);
            PCDWriter writer;
            writer.writeBinary(string(szLog), *points_target);
        }
    }

    Mat features = Mat(line_numbers.size(), 7, CV_64F, line_numbers.data()).clone();

    if (bIsSavePt)
    {
        char szLog[1024] = {0};
        sprintf(szLog, "ObjectFeature%06d.yml", nFrame);
        cv::FileStorage fs;
        fs.open(szLog, FileStorage::WRITE);
        fs << "features" << features;
        fs.release();
    }


    return features;
}

int match_clusters(std::vector<segmatch::Segment>& cluster1, std::vector<segmatch::Segment>& cluster2,
                   std::vector<segmatch::Match>& matches, CvRTrees& tree, float confidence/* = 0.7*/)
{
    matches.clear();
    for (unsigned int i = 0; i < cluster1.size(); i++)
    {
        for (unsigned int j = 0; j < cluster2.size(); j++)
        {
            Mat feature_diff = compute_eigen_features_distance(
                        cluster1[i].feature, cluster2[j].feature);
            Mat feature_diff_;
            feature_diff.convertTo(feature_diff_, CV_32F);
            float confi = tree.predict_prob(feature_diff_);
            if (confi >= confidence)
            {
                segmatch::Match match;
                match.id1 = i;
                match.id2 = j;
                match.feature1 = cluster1[i].feature.clone();
                match.feature2 = cluster2[j].feature.clone();
                match.center1 = cluster1[i].center;
                match.center2 = cluster2[j].center;
                match.confidence = confi;
                matches.push_back(match);
            }
        }
    }

    return matches.size();
}

void display_cluster_and_matches(std::vector<segmatch::Segment>& cluster1, std::vector<segmatch::Segment>& cluster2,
                                 std::vector<segmatch::Match>& matches, std::vector<int>* pvalid_mask)
{
    pcl::visualization::PCLVisualizer* p = NULL;
    int vp_1, vp_2;
    p = new pcl::visualization::PCLVisualizer("Pairwise Incremental Registration example");
//    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort (0.0, 0, 1.0, 1.0, vp_1);
    p->setCameraPosition(0.0, 0.0, 100, 0.0, 0.0, 0.0, vp_1);
//    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
//    p->setCameraPosition(0.0, 0.0, 20, 0.0, 0.0, 0.0, vp_2);
    p->addCoordinateSystem(1.0,"vp_1" , vp_1);
//    p->addCoordinateSystem(1.0,"vp_2" , vp_2);

//    std::vector<segmatch::Segment> cluster1_ = cluster1;
    for (unsigned int i = 0; i < cluster1.size(); i++)
    {
        for (unsigned int j = 0; j < cluster1[i].point_cloud.size(); j++)
        {
            cluster1[i].point_cloud[j].z = cluster1[i].point_cloud[j].z + 20.f;
        }
    }

//    p->removeAllPointClouds();
    srand (static_cast<unsigned int> (time (0)));
    for (unsigned int i = 0; i < cluster1.size(); i++)
    {
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>* handler =
                new visualization::PointCloudColorHandlerCustom<PointXYZI> (
                    cluster1[i].point_cloud.makeShared(),rand ()%255,
                    rand ()%255,rand ()%255);
        string szT = string("PointCloud1") + to_string(i);
//        p->removePointCloud(szT,vp_1);
        p->addPointCloud(cluster1[i].point_cloud.makeShared(),*handler, szT, vp_1);
        delete handler;
    }

    for (unsigned int i = 0; i < cluster2.size(); i++)
    {
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>* handler =
                new visualization::PointCloudColorHandlerCustom<PointXYZI> (
                    cluster2[i].point_cloud.makeShared(),rand ()%255,
                    rand ()%255,rand ()%255);
        string szT = string("PointCloud2") + to_string(i);
//        p->removePointCloud("1",vp_1);
        p->addPointCloud(cluster2[i].point_cloud.makeShared(),*handler, szT, vp_1);
        delete handler;
    }

    p->removeAllShapes();
    vector<int> valid_mask_t;
    if (pvalid_mask)
        valid_mask_t = *pvalid_mask;
    else
        valid_mask_t.resize(matches.size(),1);

    for (unsigned int i = 0; i < matches.size(); i++)
    {
        string szT = string("LIne") + to_string(i);
        matches[i].center1.z += 20.0;
        if (valid_mask_t[i] == 0)
        {
//            p->addLine<PointXYZ,PointXYZ>(matches[i].center1, matches[i].center2,0.,
//                                          0,0, szT);
        }
        else
        {
            p->addLine<PointXYZ,PointXYZ>(matches[i].center1, matches[i].center2,0,
                                          1,0, szT);
        }
    }

    p->spin();
}

int filter_match(std::vector<segmatch::Segment>& cluster1, std::vector<segmatch::Segment>& cluster2,
                 std::vector<segmatch::Match>& matches, std::vector<int>& valid_mask)
{
    valid_mask.resize(matches.size(), 0);
    cout << "111" << endl;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    PointCloud<PointXYZ>::Ptr first_cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr second_cloud(new PointCloud<PointXYZ>);

    for (size_t i = 0u; i < matches.size(); ++i)
    {
        // First centroid.
        PointXYZ first_centroid = matches[i].center1;
        first_cloud->push_back(first_centroid);
        // Second centroid.
        PointXYZ second_centroid = matches[i].center2;
        second_cloud->push_back(second_centroid);
        float squared_distance = 1.0 - matches[i].confidence;
        correspondences->push_back(pcl::Correspondence(i, i, squared_distance));
        cout << i <<"Point1:" << first_centroid << "Point2:" << second_centroid <<
                "squared_distance:" << squared_distance <<endl;
    }

    cout << "222" << endl;

    std::vector<Eigen::Matrix4f,
            Eigen::aligned_allocator<Eigen::Matrix4f> > correspondence_transformations;
    vector<Correspondences> clustered_corrs;
    pcl::GeometricConsistencyGrouping<PointXYZ, PointXYZ> geometric_consistency_grouping;
    geometric_consistency_grouping.setGCSize(0.4);
    geometric_consistency_grouping.setGCThreshold(4);
    geometric_consistency_grouping.setInputCloud(first_cloud);
    geometric_consistency_grouping.setSceneCloud(second_cloud);
    geometric_consistency_grouping.setModelSceneCorrespondences(correspondences);
    geometric_consistency_grouping.recognize(correspondence_transformations, clustered_corrs);

    cout << "333" << endl;
    int largest_cluster_size = 0;
    int largest_cluster_index = -1;
    for (size_t i = 0u; i < clustered_corrs.size(); ++i)
    {
        cout << "Cluster " << i << " has " << clustered_corrs[i].size() << "segments." << endl;
        if (clustered_corrs[i].size() >= largest_cluster_size)
        {
            largest_cluster_size = clustered_corrs[i].size();
            largest_cluster_index = i;
        }
    }

    std::vector<segmatch::Match> out;

    if (largest_cluster_index < 0)
    {
        cout << "No valid matches found!" << endl;
//        matches = out;
        return -1;
    }
    else
    {
        cout << "transformation:" << endl << correspondence_transformations[largest_cluster_index] << endl;
    }

    Correspondences& cos = clustered_corrs[largest_cluster_index];
    for (unsigned int i = 0; i < cos.size(); i++)
    {
        if (cos[i].index_query != cos[i].index_match)
        {
            cout << "cos[i].index_query != cos[i].index_match" << endl;
            getchar();
        }
        out.push_back(matches[cos[i].index_query]);
        valid_mask[cos[i].index_query] = 1;
    }
//    matches = out;

    cout << out.size() << " valid matches found!" << endl;

    return 1;
}
