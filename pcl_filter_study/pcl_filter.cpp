#include "pcl_filter.h"



void PCLFilterStudy::voxel_grid_filter(float lx, float ly, float lz) {
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>(cloud_));
    pcl::PointCloud<PointT>::Ptr out_ptr(new pcl::PointCloud<PointT>());
    
    // voxel grid filter
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud_ptr);
    voxel_grid.setLeafSize(lx, ly, lz);
    voxel_grid.filter(*out_ptr);
    std::cout << out_ptr->width << std::endl;

    // save
    std::string filename("/home/ray/codes/boring/data/0_pcl_voxel_grid_filtered.pcd");
    pcl::io::savePCDFileBinary(filename, *out_ptr);
}

void PCLFilterStudy::load_pcd_file(std::string& filename) {
    pcl::io::loadPCDFile(filename, cloud_);
    std::cout << cloud_.width << std::endl;
}


void PCLFilterStudy::pass_through_filter(std::string filed, float lower_limit, float upper_limit) {
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>(cloud_));
    pcl::PointCloud<PointT>::Ptr out_ptr(new pcl::PointCloud<PointT>());

    // passthrough
    pcl::PassThrough<PointT> pass_throuth;
    pass_throuth.setInputCloud(cloud_ptr);
    pass_throuth.setFilterFieldName(filed);
    pass_throuth.setFilterLimits(lower_limit, upper_limit);
    pass_throuth.filter(*out_ptr);
    std::cout << out_ptr->width << std::endl;

    // save
    std::string filename("/home/ray/codes/boring/data/0_pcl_pass_through_filtered.pcd");
    pcl::io::savePCDFileBinary(filename, *out_ptr);
}

void PCLFilterStudy::statistical_outlier_removal(int k, float stddev_mult) {
    
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>(cloud_));
    pcl::PointCloud<PointT>::Ptr out_ptr(new pcl::PointCloud<PointT>());
    
    pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    statistical_filter.setInputCloud(cloud_ptr);
    statistical_filter.setMeanK(k);
    statistical_filter.setStddevMulThresh(stddev_mult);
    statistical_filter.filter(*out_ptr);

    // save
    std::string filename("/home/ray/codes/boring/data/0_pcl_statistical_outlier_filtered.pcd");
    pcl::io::savePCDFileBinary(filename, *out_ptr);
    std::cout << out_ptr->width << std::endl;
}

void PCLFilterStudy::radius_filter(float r, int k) {
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>(cloud_));
    pcl::PointCloud<PointT>::Ptr out_ptr(new pcl::PointCloud<PointT>());

    // radius outlier removal
    pcl::RadiusOutlierRemoval<PointT> radius_removal;
    radius_removal.setInputCloud(cloud_ptr);
    radius_removal.setMinNeighborsInRadius(k);
    radius_removal.setRadiusSearch(r);
    radius_removal.filter(*out_ptr);

    // save
    std::string filename("/home/ray/codes/boring/data/0_pcl_radius_outlier_filtered.pcd");
    pcl::io::savePCDFileBinary(filename, *out_ptr);
    std::cout << out_ptr->width << std::endl;
}
