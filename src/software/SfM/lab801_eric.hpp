#pragma once
#include "lab801_allen.hpp"
#include "lab801_diana.hpp"

namespace Lab {

    // ������任������point
    cv::Point3d mapPoint(const Eigen::Matrix<double, 3, 3> &mapping, cv::Point3d &point);
    void mapPoint(const Eigen::Matrix<double, 3, 3> &mapping, vector<pair<cv::Point3d, cv::Point3d> > &pointLine);
}