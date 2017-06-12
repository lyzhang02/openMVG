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
    using vectorMat34d = std::vector<Eigen::Matrix<double, 3, 4>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4> > >; // �洢3*4 double �����vector
    using vectorMat36d = std::vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6> > >; // �洢3*6 double�����vector
    using vectorVec61d = std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1> > >; // �洢6*1 double������vector
    using vectorVec61f = vector < Eigen::Matrix<float, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 6, 1> > >; 

    /*
    @ ֱ�߻ָ� overload
    @ input   groupSize ÿ��ѡ�õ�ֱ������
    @ output  outPluckerLine plucker�����ʾ��3Dֱ��
    */
    void reconstruction_lineLinear(const SfM_Data &sfm_data, const LabData &lab_data, int groupSize,
        vectorVec61d &outPluckerLine);

    /*
    @ ֱ�߻ָ� overload
    @ input   groupSize ÿ��ѡ�õ�ֱ������
    @ output  outPointLine 3dֱ���ڿռ��е������ʾ����x1,y1,0) ��x2, y2, 1)
    */
    void reconstruction_lineLinear(const SfM_Data &sfm_data, const LabData &lab_data, int groupSize,
        vector<pair<cv::Point3d, cv::Point3d> > &outPointLine);

    /*
    @ ��3d��6�������ʾ��ֱ�� ������3d���ʾ
    @ input plukerLine ����6����ֱ��
    @ output outPointLine ÿ��ֱ�߱�ʾΪ��x1,y1,0) (x2,y2,1)
    */
    void plucker2Point(const vectorVec61d &pluckerLine, vector<pair<cv::Point3d, cv::Point3d> >& outPointLine);
}