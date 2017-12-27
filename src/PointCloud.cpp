#include "PointCloud.hpp"

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

PointCloud::PointCloud()
    : cloudPtr(new pcl::PointCloud <pcl::PointXYZRGB>)
{
}

PointCloud PointCloud::fromLAS(const std::string& filename)
{
    PointCloud pc;
	auto& cloud = *pc.cloudPtr;

	// Opening  the las file
	std::ifstream ifs(filename, std::ios::in | std::ios::binary);

	// Safeguard against opening failure
	if (ifs.fail())
	{
		std::cerr << "ERROR : Impossible to open the file : " << filePath <<std::endl;
		return;
	}

	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs); // reading las file
	unsigned long int nbPoints=reader.GetHeader().GetPointRecordsCount();

	// Fill in the cloud data
	cloud.width    = nbPoints;				// This means that the point cloud is "unorganized"
	cloud.height   = 1;						// (i.e. not a depth map)
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	cout << "INFO : " << cloud.points.size() << " points detected in " << filePath << endl;

	int i=0;				// counter
	uint16_t r1, g1, b1;	// RGB variables for .las (16-bit coded)
	int r2, g2, b2;			// RGB variables for converted values (see below)
	uint32_t rgb;			// "packed" RGB value for .pcd

	while (reader.ReadNextPoint())
	{
		// get XYZ information
		cloud.points[i].x = (reader.GetPoint().GetX());
		cloud.points[i].y = (reader.GetPoint().GetY());
		cloud.points[i].z = (reader.GetPoint().GetZ());

		// get RGB information. Note: in liblas, the "Color" class can be accessed from within the "Point" class, thus the triple gets
		r1 = (reader.GetPoint().GetColor().GetRed());
		g1 = (reader.GetPoint().GetColor().GetGreen());
		b1 = (reader.GetPoint().GetColor().GetBlue());

		// .las stores RGB color in 16-bit (0-65535) while .pcd demands an 8-bit value (0-255). Let's convert them!
		r2 = ceil(((float)r1/65536)*(float)256);
		g2 = ceil(((float)g1/65536)*(float)256);
		b2 = ceil(((float)b1/65536)*(float)256);

		// PCL particularity: must "pack" the RGB into one single integer and then reinterpret them as float
		rgb = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b2);

		cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);

		i++; // ...moving on
	}

    return pc;
}

PointCloud PointCloud::fromPCD(const std::string& filename)
{
    PointCloud pc;
	auto& cloud = pc.cloudPtr;
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (filename, *cloud) == -1)
        throw runtime_error("Reading " + filename + " failed.");
    return pc;
}