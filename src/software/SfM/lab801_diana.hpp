//#ifndef _LAB_801_DIANA_H
//#define _LAB_801_DIANA_H
//#endif // !_LAB_801_DIANA_H

#pragma once

#include "lab801_allen.hpp"
#include "opencv2/line_descriptor.hpp"
#include "eigen\Dense"
using std::vector;
using cv::line_descriptor::KeyLine;
using Eigen::Matrix;
namespace Lab {
    using vectorMat34d = std::vector<Eigen::Matrix<double, 3, 4>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4> > >; // 存储3*4 double 矩阵的vector
    using vectorMat36d = std::vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6> > >; // 存储3*6 double矩阵的vector
    using vectorVec61d = std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1> > >; // 存储6*1 double向量的vector
    using vectorVec61f = vector < Eigen::Matrix<float, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 6, 1> > >; 

    /*
    @ 直线恢复 overload
    @ input   groupSize 每组选用的直线数量
    @ output  outPluckerLine plucker坐标表示的3D直线
    */
    void reconstruction_lineLinear(const SfM_Data &sfm_data, const LabData &lab_data, int groupSize,
        vectorVec61d &outPluckerLine);

    /*
    @ 直线恢复 overload
    @ input   groupSize 每组选用的直线数量
    @ output  outPointLine 3d直线在空间中的两点表示，（x1,y1,0) （x2, y2, 1)
    */
    void reconstruction_lineLinear(const SfM_Data &sfm_data, const LabData &lab_data, int groupSize,
        vector<pair<cv::Point3d, cv::Point3d> > &outPointLine);

    /*
    @ 将3d中6个坐标表示的直线 用两个3d点表示
    @ input plukerLine 多条6坐标直线
    @ output outPointLine 每条直线表示为（x1,y1,0) (x2,y2,1)
    */
    void plucker2Point(const vectorVec61d &pluckerLine, vector<pair<cv::Point3d, cv::Point3d> >& outPointLine);
}