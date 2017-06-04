//#ifndef _LAB_801_CRYSTAL_H
//#define _LAB_801_CRYSTAL_H

/*
*   此头文件声明交互相关的函数
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
        * 需要进行图片显示，命令行输入的函数在此命名空间中
        */
        bool choose_pose(Lab::LabData& lab_data); //输入进行计算的pose号

        // 将选择的pose号更新到LabData中的chosen_to_pose
        bool initial_chosen_to_pose(Lab::LabData& lab_data, const vector<int>& pose_chosen = std::vector<int>(), bool fileOrMemory = false, const string& fileName = std::string());

        /*
        * 截图
        * 输入 winName， 显示图片的窗口名称
        * 输入 picPath， 输入图像的目录
        * 输入 pic，     输入图像的名称， picPath + pic 组成图片的绝对路径
        * 输入 outPutPath， 输出截取图像的目录，
        * outPutPath + pic.txt 组成记录截图在原图中左上角位置和宽高的数字
        * outPutPath + pic.jpg 组成截图结果的绝对路径
        */
        bool capture_picture(const string& winName, const string& picPath, const string& picName, const string& outPutPath);

       
        void draw_lines(vector<Mat>& images, vector<vector<KeyLine>>& lines);
        void draw_lines(Mat image, const vector<KeyLine> &lines);
    }
    void detect_line(vector<Mat>& images, vector<vector<KeyLine>>& lines, int longK);
}

//#endif