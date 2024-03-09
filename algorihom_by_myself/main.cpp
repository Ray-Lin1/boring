#include "fps.hpp"
#include <string>


int main() {
    std::string filename("/home/ray/codes/boring/data/0.pcd");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(filename,*cloud);

    FPSFilter fps;
    fps.setInputCloud(cloud);
    fps.setNum(2000);
    fps.filter(out);

    return 0;
}