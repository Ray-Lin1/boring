#include "utilities.h"

void vector_print(std::vector<float>& vec) {
    for (auto &x : vec) {
        std::cout << x << " ";
    }
    std::cout << "\n";
}

void vector_print_by_index(std::vector<float> &vec, std::vector<int> &indices) {
    for (auto &x: indices) {
        std::cout << vec[x] << " ";
    }
    std::cout << "\n";
}

void indices_print(std::vector<int>& vec) {
    for (auto &x : vec) {
        std::cout << x << " ";
    }
    std::cout << "\n";
}

float find_vector_mid_val(std::vector<float> &vec) {
    std::vector<float> vec_copy(vec);
    float ret = 0;
    int mid_pos = vec_copy.size() / 2;
#if DebugLevel
    std::cout << "middle position: " << mid_pos << "\t"
             <<  "vector size: " << vec_copy.size() << std::endl;
#endif
    find_vector_mid_recursion(vec_copy, ret, 0, vec.size() - 1, mid_pos);
    return ret;
}


void find_vector_mid_recursion(std::vector<float> &vec, float &val, int l, int r, int pos) {
    if (l == r && l == pos) {
        val = vec[l];
        return;
    }

    int i = l + 1, j = r;
    while (i <= j) {
        if (vec[i] <= vec[l]) {
            i++;
        } else {
            float temp = vec[i];
            vec[i] = vec[j];
            vec[j] = temp;
            j--;
        }
    }

    float temp = vec[l];
    vec[l] = vec[j];
    vec[j] = temp;
#if DebugLevel
    vector_print(vec);
#endif 
    // find the element, than return
    if (j == pos) {
        val = vec[j];
        return;
    }

    if (pos > j) {
        find_vector_mid_recursion(vec, val, j + 1, r, pos);
    } else {
        find_vector_mid_recursion(vec, val, l, j - 1, pos);
    }
}

void sort_vector_by_index(std::vector<float> &vec, std::vector<int> &indices) {
    indices.resize(vec.size());
    for (int i = 0; i < vec.size(); i++) {
        indices[i] = i;
    }
    sort_vector_by_index_recursion(vec, indices, 0, vec.size() -1);
}

void sort_vector_by_index_recursion(std::vector<float> &vec, std::vector<int> &indices, int l, int r) {
    if (l > r) return;

    int i = l + 1;
    int j = r;

    float compare = vec[indices[l]];
    while (i <= j) {
        float temp = vec[indices[i]];
        if (temp < compare) {
            i++;
        } else {
            int idx_temp = indices[j];
            indices[j] = indices[i];
            indices[i] = idx_temp;
            j--;
        }
    }

    int idx_temp = indices[j];
    indices[j] = indices[l];
    indices[l] = idx_temp;

    sort_vector_by_index_recursion(vec, indices, l, j - 1);
    sort_vector_by_index_recursion(vec, indices, j + 1, r);
}


float find_vector_mid_index(std::vector<float> &vec, std::vector<int> &indices) {
    int s = vec.size();
    indices.resize(s);
    for (int i = 0; i < s; i++) {
        indices[i] = i;
    }
    find_vector_mid_index_recursion(vec, indices, 0, s - 1, s / 2);
    return vec[indices[s / 2]];
}


void find_vector_mid_index_recursion(std::vector<float> &vec, std::vector<int> &indices, 
                                    int l, int r, int pos) {
    if (l == r && l == pos) return;

    int i = l + 1;
    int j = r;
    float compare = vec[indices[l]];
    while (i <= j) {
        float temp = vec[indices[i]];
        if (temp < compare) {
            i++;
        } else {
            int idx_temp = indices[j];
            indices[j] = indices[i];
            indices[i] = idx_temp;
            j--;
        }
    }

    int idx_temp = indices[j];
    indices[j] = indices[l];
    indices[l] = idx_temp;

    if (j == pos) return;
    if (j < pos) {
        find_vector_mid_index_recursion(vec, indices, j + 1, r, pos);
    } else {
        find_vector_mid_index_recursion(vec, indices, l, j - 1, pos);
    }
}


void find_pointcloud_mid_index_recursion(PointCloud& cloud,
                                        std::vector<int>& indices,
                                        int axis, int l, int r
                                        ) {
    if (l == r && l == indices.size() / 2) {
        return;
    }

    int i = l + 1;
    int j = r;
    float compare = ((float*)&(cloud.points_[indices[l]]))[axis];
    while (i <= j) {
        float temp = ((float*)&(cloud.points_[indices[i]]))[axis];
        if (temp < compare) {
            i++;
        } else {
            int temp = indices[j];
            indices[j] = indices[i];
            indices[i] = temp;
            j--;
        }
    }

    int temp = indices[j];
    indices[j] = indices[l];
    indices[l] = temp;
    
    if (j == indices.size() / 2) {
        return;
    }

    if (j > indices.size() / 2) {
        find_pointcloud_mid_index_recursion(cloud, indices, axis, l, j - 1);
    } else {
        find_pointcloud_mid_index_recursion(cloud, indices, axis, j + 1, r);
    }
}

void find_pointcloud_mid_index(PointCloud& cloud, int axis, std::vector<int>& indices) {
    find_pointcloud_mid_index_recursion(cloud, indices, axis, 0, indices.size() - 1);
}


