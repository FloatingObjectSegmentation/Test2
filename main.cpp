// std library
#include <iostream>
#include <vector>
#include <string>

// external dependencies
#include <liblas/liblas.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

// base classes
#include "PointCloud.hpp"
#include "Visualizer.hpp"
#include "Method.hpp"

// concrete methods
#include "RegionGrowingRGB.hpp"
#include "MinCut.hpp"
#include "ProgressiveMorphologicalFilter.hpp"

using namespace std;

int main(int argc, char* argv[]) {
	
	if (argc < 2)
	{
		cout << "Usage: " << argv[0] << " <point cloud file>" << endl;
		return 1;
	}

	PointCloud cloud = PointCloud::fromLAS(argv[1]);
	cout << "Point count: " << cloud.pointCount() << endl;

	cloud.normalizePoints();

	RegionGrowingRGB met;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg = met.computeSegmentation(cloud.getCloudPtr());

	// visualize the data
	Visualizer vs = Visualizer::rgbVisualizer(seg);
	vs.mainLoop();

	return (0);
}