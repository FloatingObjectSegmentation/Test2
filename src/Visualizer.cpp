#include "Visualizer.hpp"

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

Visualizer::Visualizer() : viewer(new boost::shared_ptr<pcl::visualization::PCLVisualizer>)
{

}


Visualizer Visualizer::simpleVisualizer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) 
{
    Visualizer vis;
	vis.viewer->setBackgroundColor(0, 0, 0);
	vis.viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	vis.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	vis.viewer->addCoordinateSystem(1.0);
	vis.viewer->initCameraParameters();
	return vis;
}

Visualizer Visualizer::rgbVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) 
{
    Visualizer vis;
	vis.viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	vis.viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	vis.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	vis.viewer->addCoordinateSystem(1.0);
	vis.viewer->initCameraParameters();
	return vis;   
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer::getViewer() 
{
    return viewer;
}