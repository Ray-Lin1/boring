#pragma once
#include "utilities.h"
#include <cmath>

struct Node {
    int axis;
    float val;
    bool is_leaf;
    Node* l;
    Node* r;
    std::vector<int> indices;
};




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

class RayKdTreeV2 {
public:
    RayKdTreeV2() {}
    ~RayKdTreeV2() {}
    void build(PointCloud& cloud, int leaf_size) {
        std::vector<int> indices;
        for(int i = 0; i < cloud.size(); i++) {
            indices.push_back(i);
        }
        head = build_recursion(cloud, indices, 0, leaf_size);
    }
    void clear() {
        clear_recursion(head);
        delete head;
        head = nullptr;
    }

    void radius_search(PointCloud& cloud,
                        PointXYZ& p,
                        RadiusSearchResult& result,
                        float r) {
        radius_search_recursion(cloud, result, p, head, r);
    }

    void knearest_search(PointCloud& cloud,
                        PointXYZ& p,
                        KNearestSearchResult& result) {
        knearest_search_recursion(cloud, result, p, head);
    }

    Node* get_head() {
        return head;
    }
private:
    Node* build_recursion(PointCloud& cloud, 
                        std::vector<int>& indices,
                        int axis,
                        int leaf_size) {
        Node* ptr = new Node;
        ptr->axis = axis;
        ptr->is_leaf = false;
        for (auto idx: indices) {
            ptr->indices.push_back(idx);
        }

        if (indices.size() <= leaf_size) {
            ptr->l = nullptr;
            ptr->r = nullptr;
            ptr->is_leaf = true;
            return ptr;
        }
        
        std::vector<int> indices_l;
        std::vector<int> indices_r;
        int half_pos = indices.size() / 2;
        find_pointcloud_mid_index(cloud, axis, indices);
        ptr->val = ((float*)&(cloud.points_[indices[half_pos]]))[axis];
        for (int i = 0; i < indices.size(); i++) {
            float temp = ((float*)&(cloud.points_[indices[i]]))[axis];
            if (temp < ptr->val) {
                indices_l.push_back(indices[i]);
            } else {
                indices_r.push_back(indices[i]);
            }
        }
        ptr->l = build_recursion(cloud, indices_l, (axis + 1) % 3, leaf_size);
        ptr->r = build_recursion(cloud, indices_r, (axis + 1) % 3, leaf_size);
        return ptr;
    } 

    void clear_recursion(Node* ptr) {
        if (ptr->l != nullptr) {
            clear_recursion(ptr->l);
            delete ptr->l;
        }
        if (ptr->r != nullptr) {
            clear_recursion(ptr->r);
            delete ptr->r;
        }
    }

    void radius_search_recursion(PointCloud& cloud,
                        RadiusSearchResult& result,
                        PointXYZ& p,
                        Node* ptr,
                        float r) {
        if (ptr->is_leaf) {
            for (auto& idx: ptr->indices) {
                float d = cal_d(p, cloud.points_[idx]);
                if (d <= r) {
                    result.update(idx, d);
                }
            }
            return;
        }

        float compare = ((float*)&p)[ptr->axis];
        if (compare < ptr->val) {
            radius_search_recursion(cloud, result, p, ptr->l, r);
            if (fabs(compare - ptr->val) < r) {
                radius_search_recursion(cloud, result, p, ptr->r, r);
            }
        } else {
            radius_search_recursion(cloud, result, p, ptr->r, r);
            if (fabs(compare - ptr->val) < r) {
                radius_search_recursion(cloud, result, p, ptr->l, r);
            }
        }
    }

    void knearest_search_recursion(PointCloud& cloud,
                        KNearestSearchResult& result,
                        PointXYZ& p,
                        Node* ptr) {
        if (ptr->is_leaf) {
            for (auto& idx: ptr->indices) {
                float d = cal_d(p, cloud.points_[idx]);
                result.update(idx, d);
                }
            return;
        }

        float compare = ((float*)&p)[ptr->axis];
        if (compare < ptr->val) {
            knearest_search_recursion(cloud, result, p, ptr->l);
            if (fabs(compare - ptr->val) < result.get_worst_distance()) {
                knearest_search_recursion(cloud, result, p, ptr->r);
            }
        } else {
            knearest_search_recursion(cloud, result, p, ptr->r);
            if (fabs(compare - ptr->val) < result.get_worst_distance()) {
                knearest_search_recursion(cloud, result, p, ptr->l);
            }
        }
    }

private:
    Node* head;
};