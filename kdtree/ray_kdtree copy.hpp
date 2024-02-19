#pragma once
#include "utilities.h"
#include <cmath>

float cal_d(PointXYZ& p1, PointXYZ& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + 
                (p1.y - p2.y) * (p1.y - p2.y) + 
                (p1.z - p2.z) * (p1.z - p2.z));
}

class KNearestSearchResult {
public:
    KNearestSearchResult(int k): size_(0), capacity_(k), worst_distance_(999.0f) {
        indices_.resize(k);
        distances_.resize(k);
    }
    ~KNearestSearchResult() {}
    void update(int idx, float d) {
        if (d > worst_distance_) {
            return;
        }
        int i;
        if (size_ < capacity_) {
            i = size_;
            size_++;
        } else {
            i = capacity_;
        }

        while (i > 0 && distances_[i - 1] > d) {
            indices_[i] = indices_[i - 1];
            distances_[i] = distances_[i - 1];
            i--;
        }
        indices_[i] = idx;
        distances_[i] = d;
        worst_distance_ = distances_[size_ - 1];
    }
    float get_worst_distance() {
        return worst_distance_;
    }
public:
    std::vector<float> distances_;
    std::vector<int> indices_;
private:
    int capacity_;
    int size_;
    float worst_distance_;
};

class RadiusSearchResult {
public:
    RadiusSearchResult(): size_(0), worst_distance_(999.0f) {}
    ~RadiusSearchResult() {}
    void update(int index, float d) {
        indices_.push_back(0);
        distance_.push_back(0.0f);;
        int i = size_;
        size_++;
        while (i > 0 && distance_[i - 1] > d) {
            indices_[i] = indices_[i - 1];
            distance_[i] = distance_[i - 1];
            i--;
        }
        distance_[i] = d;
        indices_[i] = index;
        worst_distance_ = distance_[size_ - 1];
    }

public:
    std::vector<float> distance_;
    std::vector<int> indices_;
private:
    float worst_distance_;
    int size_;
};

class RayKdTree {
public:
    RayKdTree() {}
    ~RayKdTree() {}
    void build(PointCloud& cloud, int leaf_size) {
        std::vector<int> indices;
        for (int i = 0; i < cloud.size(); i++) {
            indices.push_back(i);
        }
        build_recursion(cloud, indices, 0, leaf_size);
    }

    void build_recursion(PointCloud& cloud, 
                        std::vector<int>& indices,
                        int axis, int leaf_size) {
        axis_ = axis;
        indices_ = indices;
        if (indices.size() <= leaf_size) {
            is_leaf_ = true;
            return;
        }

        find_pointcloud_mid_index(cloud, axis, indices);

        int half_pos = indices.size() / 2;
        val_ = ((float*)&(cloud.points_[indices[half_pos]]))[axis];
        std::vector<int> indices_l;
        std::vector<int> indices_r;
        for (int i = 0; i < indices.size(); i++) {
            if (((float*)&(cloud.points_[indices[i]]))[axis] < val_) {
                indices_l.push_back(indices[i]);
            } else {
                indices_r.push_back(indices[i]);
            }
        }

        l_ = new RayKdTree();
        l_->build_recursion(cloud, indices_l, (axis + 1) % 3, leaf_size);
        r_ = new RayKdTree();
        r_->build_recursion(cloud, indices_r, (axis + 1) % 3, leaf_size);
    }

    void clear_kdtree(RayKdTree* ptr) {
        if (ptr->l_ != nullptr) {
            clear_kdtree(ptr->l_);
            delete ptr->l_;
        }
        if (ptr->r_ != nullptr) {
            clear_kdtree(ptr->r_);
            delete ptr->r_;
        }
    }

    void radius_search(PointCloud& cloud, 
                        std::vector<int>& indices,
                        PointXYZ& p,
                        RadiusSearchResult& result,
                        float r) {
        radius_search_recursion(this, cloud, indices, p, result, r);
    }

    void knearest_search(PointCloud& cloud,
                            PointXYZ& p,
                            KNearestSearchResult& result) {
        knearest_search_recursion(this, cloud, p, result);
    }
private:
    RayKdTree* l_ = nullptr;
    RayKdTree* r_ = nullptr;
    int axis_ = 0;
    float val_ = 0.0f;
    std::vector<int> indices_;
    bool is_leaf_ = false;
private:
    void radius_search_recursion(
                        RayKdTree* kdtree,
                        PointCloud& cloud, 
                        std::vector<int>& indices,
                        PointXYZ& p,
                        RadiusSearchResult& result,
                        float r) {
        if (kdtree->is_leaf_) {
            for (auto idx: kdtree->indices_) {
                float d = cal_d(p, cloud.points_[idx]);
                if (d < r) {
                    result.update(idx, d);
                }
            }
            return;
        }

        float temp = ((float*)&p)[kdtree->axis_];
        if (temp > kdtree->val_) {
            radius_search_recursion(kdtree->r_, cloud, indices, p, result, r);
            if (fabs(temp - kdtree->val_) < r) {
                radius_search_recursion(kdtree->l_, cloud, indices, p, result, r);    
            }
        } else {
            radius_search_recursion(kdtree->l_, cloud, indices, p, result, r);
            if (fabs(temp - kdtree->val_) < r) {
                radius_search_recursion(kdtree->r_, cloud, indices, p, result, r);    
            }
        }
    }

    void knearest_search_recursion(
                        RayKdTree* kdtree,
                        PointCloud& cloud, 
                        PointXYZ& p,
                        KNearestSearchResult& result) {
        if (kdtree->is_leaf_) {
            for (auto idx: kdtree->indices_) {
                float d = cal_d(p, cloud.points_[idx]);
                result.update(idx, d);
            }
            return;
        }

        float temp = ((float*)&p)[kdtree->axis_];
        if (temp > kdtree->val_) {
            knearest_search_recursion(kdtree->r_, cloud, p, result);
            if (fabs(temp - kdtree->val_) < result.get_worst_distance()) {
                knearest_search_recursion(kdtree->l_, cloud, p, result);
            }
        } else {
            knearest_search_recursion(kdtree->l_, cloud, p, result);
            if (fabs(temp - kdtree->val_) < result.get_worst_distance()) {
                knearest_search_recursion(kdtree->r_, cloud, p, result);
            }
        }
    }

};