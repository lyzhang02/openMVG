//#ifndef _LAB_801_CRYSTAL_H
//#define _LAB_801_CRYSTAL_H

/*
* ��ͷ�ļ�����������صĺ���
* ��Ҫ���ڲ����������̨��ȡ��opencv���ڽ�ͼ
*/
#pragma once
#include <vector>
#include <string>
#include "lab801_allen.hpp"
#include "opencv2/line_descriptor.hpp"
#include "opencv2/xfeatures2d.hpp"
namespace Lab {
    using cv::line_descriptor::KeyLine;
    using cv::Mat;
    namespace UserInterface {
        /*
        * ��Ҫ����ͼƬ��ʾ������������ĺ����ڴ������ռ���
        */
        bool choose_pose(Lab::LabData& lab_data); //������м����pose��

        // ��ѡ���pose�Ÿ��µ�LabData�е�chosen_to_pose
        bool initial_chosen_to_pose(Lab::LabData& lab_data, const vector<int>& pose_chosen = std::vector<int>(), bool fileOrMemory = false, const string& fileName = std::string());

        /*
        * ��ͼ
        * ���� winName�� ��ʾͼƬ�Ĵ�������
        * ���� picPath�� ����ͼ���Ŀ¼
        * ���� pic��     ����ͼ������ƣ� picPath + pic ���ͼƬ�ľ���·��
        * ���� outPutPath�� �����ȡͼ���Ŀ¼��
        * outPutPath + pic.txt ��ɼ�¼��ͼ��ԭͼ�����Ͻ�λ�úͿ�ߵ�����
        * outPutPath + pic.jpg ��ɽ�ͼ����ľ���·��
        */
        bool capture_picture(const string& winName, const string& picPath, const string& picName, const string& outPutPath);

        /*
        * ����ͼʾ���ѡ��ֱ��
        * ���� images������ֱ����ȡ��ͼ��
        * ���� lines����Ӧÿһ��ͼ��ȡ����ֱ�߼���
        * ѡ��ı�ű�����labData��
        */
        void choose_line(const vector<Mat>& images, const vector<vector<KeyLine>> &lines, LabData &labData);

        void choose_line_manually(const vector<Mat>& images, const vector<vector<KeyLine>> &lines, LabData &labData);
    }

    /*
    * ��image�ϻ���ֱ�ߣ� image�ǽ���ֱ����ȡ��ͼ��lines �ڸ�ͼ���������ֱ�߼���
    */
    void draw_lines(const Mat &image, const vector<KeyLine> &lines);
    /*
    * ����drawKeyline����ֱ��
    */
    void draw_lines(vector<Mat>& images, vector<vector<KeyLine>>& lines);

    /*
    * ֱ�߼��
    * ���� images�� ����ֱ�߼�������ͼ
    * ���� longK�� �������ֱ������
    * ��� lines�� ��Ӧÿһ��ͼ����ֱ�߼���
    */
    void detect_line(vector<Mat>& images, vector<vector<KeyLine>>& lines, int longK);
    //void detect_line(const Mat& image, vector<KeyLine> &lines, int longK);

    void line_origin_image(Lab::LabData &lab_data, const string &tempPath);
}

//#endif