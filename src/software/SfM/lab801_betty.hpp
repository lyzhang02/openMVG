//#ifndef _LAB_801_BETTY_H
//#define _LAB_801_BETTY_H

#pragma once
#include "lab801_allen.hpp"

/*
*   �����������ϣ�
*/

namespace Lab {
    namespace Helper {

        void make_dir(const string& path);  //��path���洴�����ɸ����ļ���
        bool check_matrix_right(const SfM_Data& sfm_data, Eigen::Matrix3d r, const string& path);   //��sfm_data��sence��תr�����pathĿ¼�µ�ply�ļ�
        bool check_rotation_matrix(const string& path, Eigen::Matrix3d r);  //�����ת����path
        void check_labdata_rotations(const Lab::LabData& lab_data, const string& path);   //���labdata��path
        void check_labdata_choose_pose(const Lab::LabData& lab_data, const string& path); //��� ѡ���pose�ŵ�path
        void check_line_choose(const Lab::LabData &lab_data, const string &tempPath);  //�鿴ֱ��ѡ���Ƿ���ȷ��tempPath��Ž�ͼ��Ϣ���ļ���
    }
}
//#endif // !_LAB_801_BETTY_H