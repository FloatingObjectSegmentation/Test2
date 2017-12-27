#include <liblas/liblas.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <string>
#include <iostream>
#include <exception>

class PointCloud 
{
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloudPtr;
    PointCloud();

public: 
    static PointCloud fromLAS(const std::string& filename);
    static PointCloud fromPCD(const std::string& filename);
    int pointCount();
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr getCloudPtr();
    void printFirstElement();
    void normalizePoints();

};