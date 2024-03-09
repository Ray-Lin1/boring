#pragma once
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <unordered_map>
#include <random>
#include <vector>

using PointT = pcl::PointXYZ;

// template<typename PointT>
class FPSFilter {
public:
    FPSFilter() {
        inliers_ = pcl::PointIndices::Ptr(new pcl::PointIndices());
    }
    ~FPSFilter() {}
    void setNum(uint32_t k) {
        nb_after_fps_ = k;
    }

    void setInputCloud(pcl::PointCloud<PointT>::Ptr &cloud_ptr) {
        cloud_ptr_ = cloud_ptr;
    }

    void filter(pcl::PointCloud<PointT>::Ptr &out_ptr) {
        uint32_t n = cloud_ptr_->size();
        uint32_t idx_max = 0;
        uint32_t idx_cur = get_random_num(n);
        uint32_t idx = 0;
        float d;
        float max_d;
        std::vector<bool> is_visited(n, false);
        std::vector<float> max_dis(n, std::numeric_limits<float>::max());

        std::cout << "Target Number: " << nb_after_fps_ << std::endl;
        std::cout << "PointCloud size: " << n << std::endl;
        is_visited[idx_cur] = true;
        inliers_->indices.push_back(idx_cur);
        out_ptr->resize(nb_after_fps_);
        for (int i = 0; i < nb_after_fps_; i++) {
            max_d = 0.0f;
            for (int j = 0; j < n; j++) {
                if (is_visited[j] ) continue;
                d = cal_distance(cloud_ptr_->points[idx_cur], cloud_ptr_->points[j]);
                if (d < max_dis[j]) {
                    max_dis[j] = d;
                }
                if (max_d < max_dis[j]) {
                    max_d = max_dis[j];
                    idx_max = j;
                }
            }
            idx_cur = idx_max;
            is_visited[idx_cur] = true;
            inliers_->indices.push_back(idx_cur);
        }

        // extract
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_ptr_);
        extract.setIndices(inliers_);
        extract.filter(*out_ptr);

        // save
        pcl::io::savePCDFile("/home/ray/codes/boring/data/0_fps.pcd", *out_ptr);
    }

    uint32_t get_random_num(int k) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, k);
        int random_num = dis(gen);
        return random_num;
    }

    double cal_distance(PointT p1, PointT p2) {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + 
                        (p1.y - p2.y) * (p1.y - p2.y) +
                        (p1.z - p2.z) * (p1.z - p2.z));
    }

private:
    uint32_t nb_after_fps_;
    pcl::PointCloud<PointT>::Ptr cloud_ptr_;
    pcl::PointIndices::Ptr inliers_;
};