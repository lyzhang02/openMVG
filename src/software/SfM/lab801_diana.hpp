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
    using vectorMat34d = std::vector<Eigen::Matrix<double, 3, 4>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4> > >;
    using vectorMat36d = std::vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6> > >;
    using vectorVec61d = std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1> > >;
    using vectorVec61f = vector < Eigen::Matrix<float, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 6, 1> > >;
    void reconstruction_line_Linear(const SfM_Data &sfm_data, const LabData &lab_data, int groupSize,
        vectorVec61d &outPluckerLine);
}