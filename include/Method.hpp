#pragma once

#include <liblas/liblas.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

// Base class
class Method {
public:
    // pure virtual function providing interface framework.
    virtual pcl::PointCloud <pcl::PointXYZRGB>::Ptr computeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    virtual void setWidth(int w);
    virtual void setHeight(int h);

protected:
    int width;
    int height;
};
 