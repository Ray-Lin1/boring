#include "fps.hpp"
#include "RANSAC.hpp"
#include <string>
#include <sys/time.h> 


int main() {
    std::string filename("/home/ray/codes/boring/data/0.pcd");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(filename,*cloud);

    // FPSFilter fps;
    // fps.setInputCloud(cloud);
    // fps.setNum(2000);
    // fps.filter(out);
    

    timeval s, e;
    gettimeofday(&s, nullptr);

    float A, B, C, D;
    RANSAC ransac;
    ransac.setInput(cloud);
    ransac.setParam(0.2, 0.5, 100);
    ransac.fileter(out, A, B, C, D);

    gettimeofday(&e, nullptr);
    std::cout << "Takes " 
            << (e.tv_sec - s.tv_sec) + (double)(e.tv_usec - s.tv_usec) / 1e6 << "\n" ;

    std::string out_name("/home/ray/codes/boring/data/0_ground.pcd");    
    pcl::io::savePCDFile(out_name, *out);

    return 0;
}