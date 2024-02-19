#include "pointcloud.hpp"
#include "ray_kdtree.hpp"
#include <algorithm>
#include <sys/time.h>


int main() {

    std::cout << std::fixed;
    std::cout << std::setprecision(7);

    std::string filename("../../data/0.pcd");
    std::string out("../../data/0_ascii.pcd");
    PointCloud cloud(filename);
    
    RayKdTreeV2 kdtree;
    RadiusSearchResult result;
    PointXYZ p = cloud.points_[99];
    float r = 0.3;
    kdtree.build(cloud, 50);
    kdtree.radius_search(cloud, p, result, r);
    for (auto& idx: result.indices_) {
        std::cout << idx << " ";
    }
    std::cout << "\n";
    for (auto& d: result.distance_) {
        std::cout << d << " ";
    }
    std::cout << "\n";


    for (int i = 0; i < cloud.points_.size(); i++) {
        if (cal_d(p, cloud.points_[i]) < r) {
            std::cout << i << " ";
        }
    }
    std::cout << "\n";


    KNearestSearchResult result1(5);
    kdtree.knearest_search(cloud, p, result1);
    for (auto& idx: result1.indices_) {
        std::cout << idx << " ";
    }
    std::cout << "\n";
    for (auto& d: result1.distances_) {
        std::cout << d << " ";
    }
    std::cout << "\n";
    

    kdtree.clear();
    return 0;
}
