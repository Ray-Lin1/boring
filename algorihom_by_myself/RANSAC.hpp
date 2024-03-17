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
#include <random>

using PointT = pcl::PointXYZ;

class RANSAC {
public:
    RANSAC() {}
    ~RANSAC() {}
    
    void setInput(pcl::PointCloud<PointT>::Ptr& cloud) {
        cloud_ptr = cloud;
    }

    void setParam(float d_threhhold, float n_ratio, size_t max_iter) {
        d_threshold_ = d_threhhold;
        n_ratio_ = n_ratio;
        max_iter_ = max_iter;
    }

    void fileter(pcl::PointCloud<PointT>::Ptr& out
                , float& A_best
                , float& B_best
                , float& C_best
                , float& D_best) {

        int i = 0;
        int n = cloud_ptr->points.size();
        int n_groud = 0;
        int max_groud = 0;
        float ratio = 0.0f;
        float A, B, C, D;
        float v1_x, v1_y, v1_z;
        float v2_x, v2_y, v2_z;
        
        std::vector<int> n_vec(n, 0);
        for (i = 0; i < n; i++) {
            n_vec[i] = i;
        }

        i = 0;
        std::cout << "Max Iteration: " << max_iter_ << "\t"
                    << "N_ratio: " << n_ratio_ << std::endl; 
        while (i < max_iter_ && ratio < n_ratio_) {
            std::random_shuffle(n_vec.begin(), n_vec.end());
            // std::cout << n_vec[0] << "-" << n_vec[1] << "-" << n_vec[2] << "\n";
            PointT p0 = cloud_ptr->points[n_vec[0]];
            PointT p1 = cloud_ptr->points[n_vec[1]];
            PointT p2 = cloud_ptr->points[n_vec[2]];

            v1_x = p0.x - p1.x;
            v1_y = p0.y - p1.y;
            v1_z = p0.z - p1.z;

            v2_x = p2.x - p1.x;
            v2_y = p2.y - p1.y;
            v2_z = p2.z - p1.z;

            A = v1_y * v2_z - v2_y * v1_z;
            B = v1_z * v2_x - v2_z * v1_x;
            C = v1_x * v2_y - v2_x * v1_y;
            D = -(A * p0.x + B * p0.y + C * p0.z);

            n_groud = 0;
            for (int j = 0; j < n; j++) {
                PointT p = cloud_ptr->points[j];
                float d_temp = fabs(-(A * (p.x - p0.x) + B * (p.y - p0.y) + C * (p.z - p0.z)))
                                / sqrt(A * A + B * B + C * C);
                if (d_temp < d_threshold_) {
                    n_groud++;
                }
            }

            if (n_groud > max_groud) {
                max_groud = n_groud;
                ratio = (float)max_groud / n;
                A_best = A;
                B_best = B;
                C_best = C;
                D_best = D;
            }
            i++;
        }
        // print result
        std::cout << "Iteration: " << i << "\t" 
                    << "Ratio: " << ratio << std::endl;
        std::cout << "A: " << A_best << std::endl;
        std::cout << "B: " << B_best << std::endl;
        std::cout << "C: " << C_best << std::endl;
        std::cout << "D: " << D_best << std::endl;

        // extract
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        float d;
        for (i = 0; i < n; i++) {
            PointT p = cloud_ptr->points[i];
            d = fabs(D_best + A_best * p.x + B_best * p.y + C_best * p.z) 
                    / sqrt(A_best * A_best + B_best * B_best + C_best * C_best);
            if (d < d_threshold_) {
                inliers->indices.push_back(i);
            }
        }
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_ptr);
        extract.setIndices(inliers);
        extract.filter(*out);
    }


private:
    pcl::PointCloud<PointT>::Ptr cloud_ptr;
    float n_ratio_;
    float d_threshold_;
    size_t max_iter_;
};