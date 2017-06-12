#pragma once

/*
3d�ռ��е㡢�߼�����غ���
*/
#include "lab801_allen.hpp"
#include "lab801_diana.hpp"
namespace Lab {

    /*
    @ overload
    @ �����Ա任����������
    @ input mapping 3*3 ���Ա任����
    @ input point 3d��
    @ output ���ú��Point3d
    */
    cv::Point3d mapPoint(const Eigen::Matrix<double, 3, 3> &mapping, cv::Point3d &point);
    
    /*
    @ overload
    @ �������Ա任mapping�� pointLine�е�ÿһ����
    @ input mapping 3*3 �任����
    @ input&output pointLine ÿ��pair�е��������ʾ3dֱ�ߵ������յ�
    */
    void mapPoint(const Eigen::Matrix<double, 3, 3> &mapping, vector<pair<cv::Point3d, cv::Point3d> > &pointLine);

    /*
    @ ���㸩���ǶȺ�ˮƽ�Ƕ�
    @ input pointLine ����vector<pair<Point3d, Point3d>> ÿ��pair�б�ʾ3d�ռ�ֱ�ߵ������˵�
    @ path ����Ƕȵ��ļ�����
    */
    void computeAngle(const vector<pair<cv::Point3d, cv::Point3d> >& pointLine, const string &path);
}