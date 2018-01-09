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
#include <vector>
#include <iostream>
#include <exception>

class PointCloud 
{
    // main points
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloudPtr;
    
    // additional LiDAR attributes
    std::vector<uint16_t> intensities;
    std::vector<uint16_t> returnNumbers;
    std::vector<uint16_t> numbersOfReturns;
    std::vector<uint16_t> scanDirections;
    std::vector<uint8_t> scanAngleRanks;
    std::vector<uint16_t> flightEdgeLines;

public: 
    PointCloud();
    static PointCloud fromLAS(const std::string& filename);
    static PointCloud fromPCD(const std::string& filename);
    int pointCount();
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr getCloudPtr();
    pcl::PointCloud <pcl::PointXYZI> getIntensityCloudPtr();
    void printFirstElement();
    void normalizePoints();

};