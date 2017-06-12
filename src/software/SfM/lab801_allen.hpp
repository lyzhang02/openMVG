//#ifndef _LAB_801_ALLEN_H
//#define _LAB_801_ALLEN_H

#pragma once

/*
* 正交旋转矩阵计算相关函数在此头文件内
* 包括
*   读取sfm估计的pose，读取文本记录的sensor信息
*   两组矩阵的合成， 矩阵到eular角度的分解
*   eular角度合成单位正交矩阵
*/

#include "openMVG\sfm\sfm.hpp"
#include "openMVG\sfm\sfm_data.hpp"
#include "vector"
#include "unordered_map"
#include "unordered_set"
#include "map"
#include "fstream"
#include "iostream"
#include <memory>
#include <opencv2/line_descriptor.hpp>

using namespace openMVG::sfm;
using namespace std;
namespace Lab {
    class LabData {
    public:
            //图片目录的上级目录
        string root_path;
            //图片的目录
        string image_path;
            // 平均后的正交旋转矩阵
        Eigen::Matrix<double, 3, 3> mapping;
         
        int validPose;
            //旋转平移计算成功的pose编号 与 计算的旋转映射
        unordered_map<int, Eigen::Matrix<double, 3, 3>> pose_sfm_rotation;
            //计算成功的pose编号 与 传感器信息的映射
        unordered_map<int, Eigen::Matrix<double, 3, 3>> pose_sensor_rotation;

            //选定直线的pose和其camera矩阵
        map<int, Eigen::Matrix<double, 3, 4>, std::less<int>,
            Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix<double, 3, 4> > > > pose_P;
            //选择进行后续计算的pose从0开始序号 与 原始pose编号的映射
        //map<int, int> chosen_to_pose;
        set<int> chosen_to_pose;
            //选择进行后续计算的pose从0开始序号（同chosen_to_pose的key，不是原始pose编号），与 选定直线编号之间的映射
        unordered_map<int, cv::line_descriptor::KeyLine> pose_line;
            //计算成功的pose编号 与 图片名称之间的映射
        unordered_map<int, string> pose_name;
            //计算成功的pose编号 与 传感器 GPS 的映射
        unordered_map<int, Eigen::Vector3d> pose_gps;

    };
    
    /*
    * 收集需要的信息，用LabData统一起来，便于后续操作
    */
    LabData createLabData(const SfM_Data& sfm_data);
    
    //Read data txt in fileDirPath, store them in object's member.
    bool read_flight_file(LabData& lab_data, const string& fileDirPath, const string& tag1, const string& tag2);

    void read_P(LabData &lab_data, const SfM_Data &sfm_data);
    

    //Read from a single txt, tag1 stands for angle label, tag2 stands for GPS label.
    bool getSensorFromFile(const string& fileName,
        const string& tag1, const string& tag2,
        Eigen::Matrix<double, 3, 3> &rotation, Eigen::Vector3d& GPS);
    Eigen::Matrix3d compute_mapping(const LabData& lab_data);


}
//#endif // !_LAB_801_H
