#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include "pcd_header.h"

struct PointXYZ {
    float x;
    float y;
    float z;
};


class PointCloud {
public:
    PointCloud(std::string filename) {
        read_point_from_file(filename);
    }
    ~PointCloud() {}

    void save_pcd(std::string filename) {
        std::ofstream out(filename, std::ios_base::out);
        if (! out.is_open()) {
            std::cout << "Save Pcd: Open Error\n";
            exit(-1);
        }
        
        char header[2048];
        memset(header, 0, 2048);
        sprintf(header, pcd_header_str,  n_points_, n_points_);
        std::string header_str(header);
        out << header_str;
        out << std::fixed;
        out << std::setprecision(4);
        for (auto& p: points_) {
            out << p.x << " "
                << p.y << " "
                << p.z << "\n";
        }
    }

    uint32_t size() {
        return n_points_;
    }



private:
    void read_point_from_file(std::string filename) {
        std::ifstream in(filename, 
                    std::ios_base::in | std::ios_base::binary);
        if (! in.is_open()) {
            std::cout << "Open Error\n";
            exit(-1);
        }

        int n_points = 0;
        int i = 0;
        std::string line, temp;
        std::stringstream ss;

        while (! in.eof() && i < 11) {
            std::getline(in, line);
            if (line.find("WIDTH") != std::string::npos) {
                ss << line;
                ss >> temp;
                ss >> n_points;
                std::cout << n_points << "\n";
            }
            i++;
        }   

        points_.resize(n_points);
        n_points_ = n_points;
        i = 0;
        PointXYZ p;
        char *p_data = new char[n_points_ * 16];
        in.read(p_data, n_points * 16);
        for (int i = 0; i < n_points_; i++) {
            memcpy(&p, p_data + i * 16, sizeof(p));
            points_[i] = p;
        }
        delete [] p_data;
    }

private:
    uint32_t n_points_;
public:
    std::vector<PointXYZ> points_;
};
