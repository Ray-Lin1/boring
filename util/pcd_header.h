#ifndef PCD_HEADER_H
#define PCD_HEADER_H


const char pcd_header_str[] = "# .PCD v0.7 - Point Cloud Data file format\n\
VERSION 0.7\n\
FIELDS x y z\n\
SIZE 4 4 4\n\
TYPE F F F\n\
COUNT 1 1 1\n\
WIDTH %u\n\
HEIGHT 1\n\
VIEWPOINT 0 0 0 1 0 0 0\n\
POINTS %u\n\
DATA ascii\n";

#endif