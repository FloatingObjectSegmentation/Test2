// http://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.php#progressive-morphological-filtering
#include "Method.hpp"
#include "ProgressiveMorphologicalFilter.hpp"

#include <liblas/liblas.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

pcl::PointCloud <pcl::PointXYZRGB>::Ptr ProgressiveMorphologicalFilter::computeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cldrgb) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*cldrgb, *cloud);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud (cloud);
    pmf.setMaxWindowSize (20);
    pmf.setSlope (1.0f);
    pmf.setInitialDistance (0.5f);
    pmf.setMaxDistance (3.0f);
    pmf.extract (ground->indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (ground);
    extract.filter (*cloud_filtered);

    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    // Extract non-ground returns
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    std::cerr << "Object cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result;
    result = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    copyPointCloud(*cloud_filtered, *result);

    return result;
}