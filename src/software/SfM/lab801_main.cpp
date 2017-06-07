#include "third_party\cmdLine\cmdLine.h"
#include "openMVG\cameras\cameras.hpp"
#include "openMVG\sfm\sfm.hpp"
#include "openmvg\geometry\rigid_transformation3D_srt.hpp"
#include "openMVG\cameras\Cameras_Common_command_line_helper.hpp"
#include "openMVG\system\timer.hpp"
#include "openMVG\multiview\triangulation_nview.hpp"

#include "opencv2/line_descriptor.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "lab801_allen.hpp"
#include "lab801_betty.hpp"
#include "lab801_crystal.hpp"
#include "lab801_diana.hpp"
using namespace openMVG::sfm;
using cv::line_descriptor::KeyLine;
using std::vector;
using std::string;
using cv::Mat;

int main(int argc, char **argv) {
    CmdLine cmd;
    std::string sSfM_Data_Filename;
    string tag;
    double distanceBetweenTwoImages = 0.5;
    cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
    cmd.add(make_option('d', distanceBetweenTwoImages, "distanceBetweenTwoImages"));
    cmd.add(make_option('t', tag, "tag"));
    try {
        if (argc == 1) throw std::string("Invalid parameter.");
        cmd.process(argc, argv);
    }
    catch (const std::string& s) {
        std::cerr << "Usage: " << argv[0] << '\n'
            << "[-i|--input_file] path to a SfM_Data scene\n"
            << std::endl;
        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    //加载SfM_Data
    SfM_Data my_sfm_data;
    if (!Load(my_sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
        std::cerr << std::endl
            << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
        return EXIT_FAILURE;
    }
    
    // 从SfM_Data中收集需要的信息
    // pose编号， 单位正交矩阵，图像名称， 目录等
    Lab::LabData lab_data = Lab::createLabData(my_sfm_data);
    Lab::read_flight_file(lab_data, lab_data.image_path, "ZYX", "GPS"); // 从image文件夹下读取 传感器信息
    Eigen::Matrix3d m = Lab::compute_mapping(lab_data); // 将传感器信息和SfM_Data中pose的旋转合成得到到East-North-Up坐标系映射
    lab_data.mapping = m;
    Lab::Helper::make_dir(lab_data.root_path);  //创建文件夹， result存储最终结果， temp保存中间生成文件

    {   // 输出 旋转映射m
        Lab::Helper::check_rotation_matrix(lab_data.root_path + "temp/r.txt", m);
        // 输出 将point cloud旋转m 检查映射的正确性
        Lab::Helper::check_matrix_right(my_sfm_data, m, lab_data.root_path + "temp/check.ply");
        // 输出 LabData中的传感器信息和收集的SfM_Data信息，检查正确性
        Lab::Helper::check_labdata_rotations(lab_data, lab_data.root_path + "temp/ro.txt");
    }

    Lab::UserInterface::choose_pose(lab_data);  // 选择进行截图的pose编号
    
    {   //检测pose编号对应的正确性
        Lab::Helper::check_labdata_choose_pose(lab_data, lab_data.root_path + "temp/pose.txt");
    }


    /*
    *  ------------截图过程，截取后的图片放在temp文件夹-----------
    */
    const string tempPath = lab_data.root_path + "temp/"; //temp文件夹，用于存放中间生成图片，文本等
    cv::namedWindow("show", cv::WINDOW_NORMAL); //截图窗口
    //对选择的pose对应的图像进行截图
    for (const auto& k : lab_data.chosen_to_pose) {
        //const auto iter = lab_data.pose_name.find(k.second);
        const auto iter = lab_data.pose_name.find(k);
        Lab::UserInterface::capture_picture("show", lab_data.image_path, iter->second, tempPath);
    }
    cv::destroyWindow("show");
    // ---------------------！截图------------------------

    // ------------------直线检测过程---------------------
    vector<Mat> images; //images 存储截图后的子图片
    images.reserve(lab_data.chosen_to_pose.size());
    for (const auto& k : lab_data.chosen_to_pose) {
        //const auto iter = lab_data.pose_name.find(k.second);
        const auto iter = lab_data.pose_name.find(k);
        images.push_back(cv::imread(tempPath + iter->second + ".jpg"));
    }

    vector<vector<KeyLine>> lines; //lines存储images中每一张截图中检测到的直线
    Lab::detect_line(images, lines, 10); //对images中的每一张图进行直线检测，保留前K条长直线，保存在lines
    // 把直线检测的结果绘制到截图上
    for (int i = 0; i < images.size(); ++i) {
        Lab::draw_lines(images[i], lines[i]);
    }

    // 观察绘制的直线，选择每张图上对应直线
    Lab::UserInterface::choose_line(images, lines, lab_data);
    Lab::line_origin_image(lab_data, lab_data.root_path + "temp/");
    //{// 将选择的直线绘制到原始大图片上，检测是否对应正确
    //    Lab::Helper::check_line_choose(lab_data, lab_data.root_path + "temp/");
    //}
    // ----------------！直线检测过程-----------------

    vector<Mat>{}.swap(images); //清空
    vector<vector<KeyLine>>{}.swap(lines);

    Lab::read_P(lab_data, my_sfm_data);
    //vector<Matrix<double, 6, 1 >, Eigen::aligned_allocator<Matrix<double, 6, 1> > > outPluckerLine;
    Lab::vectorVec61d outPluckerLine;
    Lab::reconstruction_line_Linear(my_sfm_data, lab_data, 3, outPluckerLine);
    return 0;
}