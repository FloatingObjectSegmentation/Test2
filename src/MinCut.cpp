#include "Method.hpp"
#include "MinCut.hpp"

#include <liblas/liblas.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

using namespace pcl;

PointCloud <PointXYZRGB>::Ptr MinCut::computeSegmentation(PointCloud<PointXYZRGB>::ConstPtr cldrgb) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

    copyPointCloud(*cldrgb, *cloud);

    IndicesPtr indices (new std::vector <int>);
    PassThrough<PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    MinCutSegmentation<PointXYZ> seg;
    seg.setInputCloud (cloud);
    seg.setIndices (indices);

    PointCloud<PointXYZ>::Ptr foreground_points(new PointCloud<PointXYZ> ());
    PointXYZ point;
    point.x = 68.97;
    point.y = -18.55;
    point.z = 0.57;
    foreground_points->points.push_back(point);
    seg.setForegroundPoints (foreground_points);

    seg.setSigma (0.25);
    seg.setRadius (3.0433856);
    seg.setNumberOfNeighbours (14);
    seg.setSourceWeight (0.8);

    std::vector <PointIndices> clusters;
    seg.extract (clusters);

    std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

    PointCloud <PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
    return colored_cloud;
}
