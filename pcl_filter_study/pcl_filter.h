#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Core>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

using PointT = pcl::PointXYZ;
class PCLFilterStudy {
public:
    PCLFilterStudy() {}
    ~PCLFilterStudy() {}
    void load_pcd_file(std::string& filename);
    void voxel_grid_filter(float lx, float ly, float lz);
    void pass_through_filter(std::string filed, float lower_limit, float upper_limit);
    void statistical_outlier_removal(int k, float stddev_mult);
    void radius_filter(float r, int k);

private:
    pcl::PointCloud<PointT> cloud_;
};


