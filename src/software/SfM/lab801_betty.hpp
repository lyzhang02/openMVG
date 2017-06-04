//#ifndef _LAB_801_BETTY_H
//#define _LAB_801_BETTY_H

#pragma once
#include "lab801_allen.hpp"

/*
*   辅助函数集合，
*/

namespace Lab {
    namespace Helper {

        void make_dir(const string& path);  //在path下面创建若干辅助文件夹
        bool check_matrix_right(const SfM_Data& sfm_data, Eigen::Matrix3d r, const string& path);   //将sfm_data的sence旋转r输出到path目录下的ply文件
        bool check_rotation_matrix(const string& path, Eigen::Matrix3d r);  //输出旋转矩阵到path
        void check_labdata_rotations(const Lab::LabData& lab_data, const string& path);   //输出labdata到path
        void check_labdata_choose_pose(const Lab::LabData& lab_data, const string& path); //输出 选择的pose号到path
        void check_line_choose(const Lab::LabData &lab_data, const string &tempPath);  //查看直线选择是否正确，tempPath存放截图信息的文件夹
    }
}
//#endif // !_LAB_801_BETTY_H