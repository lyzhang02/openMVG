#pragma once

/*
3d空间中点、线计算相关函数
*/
#include "lab801_allen.hpp"
#include "lab801_diana.hpp"
namespace Lab {

    /*
    @ overload
    @ 将线性变换作用于向量
    @ input mapping 3*3 线性变换矩阵
    @ input point 3d点
    @ output 作用后的Point3d
    */
    cv::Point3d mapPoint(const Eigen::Matrix<double, 3, 3> &mapping, cv::Point3d &point);
    
    /*
    @ overload
    @ 作用线性变换mapping于 pointLine中的每一个点
    @ input mapping 3*3 变换矩阵
    @ input&output pointLine 每个pair中的两个点表示3d直线的起点和终点
    */
    void mapPoint(const Eigen::Matrix<double, 3, 3> &mapping, vector<pair<cv::Point3d, cv::Point3d> > &pointLine);

    /*
    @ 计算俯仰角度和水平角度
    @ input pointLine 类型vector<pair<Point3d, Point3d>> 每个pair中表示3d空间直线的两个端点
    @ path 输出角度的文件名称
    */
    void computeAngle(const vector<pair<cv::Point3d, cv::Point3d> >& pointLine, const string &path);
}