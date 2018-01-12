#include "SACSegmentation.hpp"

#include <iostream>
#include <vector>
#include <string>

#include <liblas/liblas.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

pcl::PointCloud <pcl::PointXYZRGB>::Ptr SACSegmentation::computeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cld) {
    
    // copy to appropriate form
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*cld, *cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    
    // Optional
    seg.setOptimizeCoefficients (true);
    
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return NULL;
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (size_t i = 0; i < inliers->indices.size (); ++i)
        std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                << cloud->points[inliers->indices[i]].y << " "
                                                << cloud->points[inliers->indices[i]].z << std::endl;


    pcl::PointCloud<pcl::PointXYZRGB> ret;

    // Fill in the cloud data
	ret.width    = inliers->indices.size();	// This means that the point cloud is "unorganized"
	ret.height   = 1;						// (i.e. not a depth map)
	ret.is_dense = false;
	ret.points.resize(inliers->indices.size());

    for (int i = 0; i < inliers->indices.size(); i++) 
    {
        ret.points[i].x = cloud->points[inliers->indices[i]].x;
        ret.points[i].y = cloud->points[inliers->indices[i]].y;
        ret.points[i].z = cloud->points[inliers->indices[i]].z;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr retPtr;
    retPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    copyPointCloud(ret, *retPtr);
    
    return retPtr;
}