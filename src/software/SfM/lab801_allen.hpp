//#ifndef _LAB_801_ALLEN_H
//#define _LAB_801_ALLEN_H

#pragma once

/*
* ������ת���������غ����ڴ�ͷ�ļ���
* ����
*   ��ȡsfm���Ƶ�pose����ȡ�ı���¼��sensor��Ϣ
*   �������ĺϳɣ� ����eular�Ƕȵķֽ�
*   eular�ǶȺϳɵ�λ��������
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
            //ͼƬĿ¼���ϼ�Ŀ¼
        string root_path;
            //ͼƬ��Ŀ¼
        string image_path;
            // ƽ�����������ת����
        Eigen::Matrix<double, 3, 3> mapping;
         
        int validPose;
            //��תƽ�Ƽ���ɹ���pose��� �� �������תӳ��
        unordered_map<int, Eigen::Matrix<double, 3, 3>> pose_sfm_rotation;
            //����ɹ���pose��� �� ��������Ϣ��ӳ��
        unordered_map<int, Eigen::Matrix<double, 3, 3>> pose_sensor_rotation;

            //ѡ��ֱ�ߵ�pose����camera����
        map<int, Eigen::Matrix<double, 3, 4>, std::less<int>,
            Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix<double, 3, 4> > > > pose_P;
            //ѡ����к��������pose��0��ʼ��� �� ԭʼpose��ŵ�ӳ��
        //map<int, int> chosen_to_pose;
        set<int> chosen_to_pose;
            //ѡ����к��������pose��0��ʼ��ţ�ͬchosen_to_pose��key������ԭʼpose��ţ����� ѡ��ֱ�߱��֮���ӳ��
        unordered_map<int, cv::line_descriptor::KeyLine> pose_line;
            //����ɹ���pose��� �� ͼƬ����֮���ӳ��
        unordered_map<int, string> pose_name;
            //����ɹ���pose��� �� ������ GPS ��ӳ��
        unordered_map<int, Eigen::Vector3d> pose_gps;

    };
    
    /*
    * �ռ���Ҫ����Ϣ����LabDataͳһ���������ں�������
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
