#ifndef UTILITIES_H
#define UTILITIES_H 
#include <vector>
#include <iostream>
#include "pointcloud.hpp"

#define DebugLevel 1

float find_vector_mid_index(std::vector<float> &vec, std::vector<int> &indices);
void find_vector_mid_index_recursion(std::vector<float> &vec, std::vector<int> &indices, 
                                    int l, int r, int pos);

float find_vector_mid_val(std::vector<float> &vec);
void find_vector_mid_recursion(std::vector<float> &vec, float &val, int l, int r, int pos);

void sort_vector_by_index(std::vector<float> &vec, std::vector<int> &indices);
void sort_vector_by_index_recursion(std::vector<float> &vec, std::vector<int> &indices, int l, int r);

void indices_print(std::vector<int>& vec);
void vector_print(std::vector<float>& vec);
void vector_print_by_index(std::vector<float> &vec, std::vector<int> &indices);

void find_pointcloud_mid_index(PointCloud& cloud, int axis, std::vector<int>& indices);

#endif