#include "pcl_filter.h"

int main() {

    PCLFilterStudy s;
    std::string filename("/home/ray/codes/boring/data/0.pcd");
    s.load_pcd_file(filename);
    // s.voxel_grid_filter(1.0f, 1.0f, 1.0f);
    // s.pass_through_filter("x", -30, 30);
    // s.statistical_outlier_removal(10, 1.0f);
    s.radius_filter(0.5, 2);

    return 0;
}