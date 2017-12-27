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

class Visualizer
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    Visualizer();

public: 
    static Visualizer simpleVisualizer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    static Visualizer rgbVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer();
};