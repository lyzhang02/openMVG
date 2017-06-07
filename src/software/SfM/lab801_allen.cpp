#include "lab801_allen.hpp"

using namespace openMVG::sfm;
using namespace std;

namespace Lab{
    Lab::LabData Lab::createLabData(const SfM_Data& sfm_data) {
        /*
        *读取sfm_data记录 重建成功的编号，与编号对应的旋转， 图像名称
        */
        Lab::LabData res;
        res.validPose = sfm_data.poses.size();
        res.image_path = sfm_data.s_root_path + "/";
        res.root_path = res.image_path + "/../";
        //记录下pose的编号，对应的旋转矩阵，对应的图片名称
        for (Poses::const_iterator iter = sfm_data.poses.cbegin(); iter != sfm_data.poses.cend(); ++iter) {
            res.pose_sfm_rotation.insert({ iter->first, iter->second.rotation() });
            res.pose_name.insert({ iter->first, sfm_data.views.at(iter->first)->s_Img_path });
        }
        if (res.pose_name.size() < 3)
            exit(1);
        return res;
    }

    /*
    *   具体读取文件fileName内的信息
    *   输入： fileName， 文件名称
    *           tag1, tag2, 标识信息开始的字符串
    *   输出： rotation 读取到的矩阵
    *           GPS 读取到的GPS
    */
    bool Lab::getSensorFromFile(const string& fileName, const string& tag1, const string& tag2, Eigen::Matrix<double, 3, 3> &res, Eigen::Vector3d& GPS) {
        ifstream read_in(fileName);
        if (!read_in.is_open()) {
            std::cerr << "open file error in getSensorFromFile.\n";
            return false;
        }
        string temp;
        //Eigen::Matrix<double, 3, 3> res;
        while (read_in >> temp) {
            //read (3,3) matrix ZYX
            if (temp == tag1) {
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        read_in >> res(i, j);
                    }
                }
            }
            if (temp == tag2) {
                for (int i = 0; i < 3; ++i) {
                    string t;
                    read_in >> t;
                    if(t != "NaN")
                    GPS(i) = std::stod(t);
                }
                if (!read_in.good()) {
                    read_in.clear();
                }
            }
        }
        read_in.close();
        return true;;
    }

    /*
    *   从与图片相关的txt内读取飞行器朝向信息
    *   输入： filePath，txt所在的目录
    *           tag1 tag2， txt内标识信息开始的字符串
    *   输出： lab_data， 信息写入lab_data内的pose_sensor_rotation 和 pose_gps两个哈希map
    */
    bool Lab::read_flight_file(Lab::LabData& lab_data,
        const string& filePath, const string& tag1, const string& tag2) {
        for (const auto& e : lab_data.pose_name) {
            Eigen::Vector3d GPS;
            Eigen::Matrix<double, 3, 3> rotation;
            //read sensor data rotation matrix and GPS from file
            if (!Lab::getSensorFromFile(filePath + "/" + e.second + ".txt", tag1, tag2, rotation, GPS)) {
                std::cerr << "read sensor file error in read_flight_file.\n";
                return false;
            }

            lab_data.pose_sensor_rotation.insert({ e.first, rotation });
            lab_data.pose_gps.insert({ e.first, GPS });
        }
        return true;
    }

    /*
    *   将单位正交矩阵转化为三个角度， 参考方法https://www.learnopencv.com/rotation-matrix-to-euler-angles/
    *   输入： r 一个代表旋转的单位正交矩阵
    *   输出： Vector3f 3元素向量，分别代表下x,y,z轴的角度，弧度制表示
    */
    Eigen::Vector3d ro2eular(const Eigen::Matrix3d& r) {
        Eigen::Vector3d vec;
        float sy = sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0));
        bool singular = (sy < 1e-6);
        if (!singular) {// x, y, z
            vec(0) = std::atan2(r(2, 1), r(2, 2)); 
            vec(1) = std::atan2(-r(2, 0), sy);
            vec(2) = std::atan2(r(1, 0), r(0, 0));
        }
        else {
            vec(0) = std::atan2(-r(1, 2), r(1, 1));
            vec(1) = std::atan2(-r(2, 0), sy);
            vec(2) = 0;
        }
        return vec;
    }

    /*
    *   从三个角度合成单位正交矩阵，按照顺序Z-Y-X(yaw, pitch, roll)
    *   输入： vec三元素向量，分别代表x,y,z旋转角度，弧度表示
    *   输出： 合成的单位正交矩阵
    *
    */
    Eigen::Matrix3d get_rotation_matrix(const Eigen::Vector3d& vec, int axis = 0) {
        double cos_x = std::cos(vec(0));
        double sin_x = std::sin(vec(0));
        double cos_y = std::cos(vec(1));
        double sin_y = std::sin(vec(1));
        double cos_z = std::cos(vec(2));
        double sin_z = std::sin(vec(2));

        Eigen::Matrix3d x;
        x << 1, 0, 0,
            0, cos_x, -sin_x,
            0, sin_x, cos_x;
        Eigen::Matrix3d y;
        y << cos_y, 0, sin_y,
            0, 1, 0,
            -sin_y, 0, cos_y;
        Eigen::Matrix3d z;
        z << cos_z, -sin_z, 0,
            sin_z, cos_z, 0,
            0, 0, 1;

        Eigen::Matrix3d m =  z * y * x;
        return m;
    }

    /*
    * 对旋转矩阵进行平均计算，此平均指简单的算数平均
    * 输入：所有旋转矩阵matrix_r，所有矩阵应该比较相近 outFile如果不为空，那么将分解后角度写入outFile的文件中
    * 输出：平均后的旋转矩阵
    * 方法：对所有矩阵进行分解计算三个eular角度
    *       将角度平均之后再合成单位正交矩阵
    */
    Eigen::Matrix3d average_rotation(const unordered_map<int, Eigen::Matrix3d>& matrix_r, const string& outFile) {
        Eigen::Vector3d vec(0.0, 0.0, 0.0);
        vector<Eigen::Vector3d> t;
        t.reserve(matrix_r.size());
        for (const auto& e : matrix_r) {
            //Eigen::Vector3d tmp = ro2eular(e.second);
            //vec += tmp;
            t.push_back(Lab::ro2eular(e.second));
        }

        for (const auto& e : t) {
            vec += e;
        }
        int s = matrix_r.size() > 0 ? matrix_r.size() : 1;
        vec /= s;
     
        if (!outFile.empty()) {
            ofstream out(outFile);
            if (out.is_open()) {
                Eigen::Vector3d square_sum(0.0, 0.0, 0.0);
                for (const auto& e : t) {
                    square_sum(0) += (e(0) - vec(0)) * (e(0) - vec(0));
                    square_sum(1) += (e(1) - vec(1)) * (e(1) - vec(1));
                    square_sum(2) += (e(2) - vec(2)) * (e(2) - vec(2));
                    out << e.transpose() << '\n';
                }
                square_sum /= s;
                out << "var: " << square_sum.transpose();
            }
            out.close();
        }
        return get_rotation_matrix(vec);
    }

    /*
    * 将传感器的朝向矩阵和图像Pose的朝向矩阵连接起来
    * 输入：lab_data  管理用到到数据的对象 pose_sfm_rotations & pose_sensor_rotation
    * 输出：Matrix3d 3*3 矩阵
    */
    Eigen::Matrix3d compute_mapping(const LabData& lab_data) {
        unordered_map<int, Eigen::Matrix<double, 3, 3> > product_rotation;
        //unordered_map<int, Eigen::Vector3f> decomposition_eular;
        for (const auto& e : lab_data.pose_sfm_rotation) {
            auto iter = lab_data.pose_sensor_rotation.find(e.first);
            Eigen::Matrix<double, 3, 3> tmp = (iter->second) * e.second;
            product_rotation.insert({ e.first, tmp });
        }

        Eigen::Matrix3d m = average_rotation(product_rotation, lab_data.root_path + "temp/decomposition.txt");
        return m;
    }

    /*
    * 收集 选定直线的pose的P矩阵信息 
    */

    void read_P(LabData &lab_data, const SfM_Data &sfm_data) {

        for (const auto &e : lab_data.pose_line) {
            //std::shared_ptr<openMVG::sfm::View> view = sfm_data.GetViews().at(originPose);
            std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic = sfm_data.GetIntrinsics().at(e.first);
            //openMVG::geometry::Pose3 pose = sfm_data.GetPoseOrDie(view.get());
            openMVG::geometry::Pose3 pose = sfm_data.poses.at(e.first);
            openMVG::Mat34 p = intrinsic->get_projective_equivalent(pose);
            lab_data.pose_P.insert({ e.first, p}); //插入pose编号，对应的camera matrix
        }
    }
}