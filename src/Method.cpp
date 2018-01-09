#include "Method.hpp"
#include <iostream>

pcl::PointCloud <pcl::PointXYZRGB>::Ptr Method::computeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    std::cout << "Null";
    return NULL;
}

void Method::setWidth(int w) {
    width = w;
}
void Method::setHeight(int h) {
    height = h;
}