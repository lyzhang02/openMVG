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

    std::cout << "hello";
    SfM_Data my_sfm_data;
    if (!Load(my_sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
        std::cerr << std::endl
            << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
        return EXIT_FAILURE;
    }
    Lab::LabData lab_data = Lab::createLabData(my_sfm_data);
    Lab::read_flight_file(lab_data, lab_data.image_path, "ZYX", "GPS");
    Eigen::Matrix3d m = Lab::compute_mapping(lab_data);
    Lab::Helper::make_dir(lab_data.root_path);

    {
        Lab::Helper::check_rotation_matrix(lab_data.root_path + "temp/r.txt", m);
        Lab::Helper::check_matrix_right(my_sfm_data, m, lab_data.root_path + "temp/check.ply");
        Lab::Helper::check_labdata_rotations(lab_data, lab_data.root_path + "temp/ro.txt");
    }

    Lab::UserInterface::choose_pose(lab_data);
    
    {
        Lab::Helper::check_labdata_choose_pose(lab_data, lab_data.root_path + "temp/pose.txt");
    }


    /*
    *  截图过程，截取后的图片放在temp文件夹
    */
    const string tempPath = lab_data.root_path + "temp/";
    cv::namedWindow("show", cv::WINDOW_NORMAL);
    for (const auto& k : lab_data.chosen_to_pose) {
        const auto iter = lab_data.pose_name.find(k.second);
        Lab::UserInterface::capture_picture("show", lab_data.image_path, iter->second, tempPath);
    }
    cv::destroyWindow("show");
    // ！截图

    // 直线检测过程
    vector<Mat> images;
    images.reserve(lab_data.chosen_to_pose.size());
    for (const auto& k : lab_data.chosen_to_pose) {
        const auto iter = lab_data.pose_name.find(k.second);
        images.push_back(cv::imread(tempPath + iter->second + ".jpg"));
    }

    vector<vector<KeyLine>> lines;
    Lab::detect_line(images, lines, 10);
    // ！直线检测过程
    return 0;
}