#include <iostream>
#include <fstream>
#include "lab801_betty.hpp"

namespace Lab {
    namespace Helper {

        /*
        *   �ڸ�Ŀ¼�´������ļ���
        *   result ��Ž��
        *   temp ������ڼ����Ե��ļ�
        */
        void make_dir(const string& path) {
            string outPut = path + "result";
            string check = path + "/temp";
            std::cout << "\n" << outPut << std::endl;
            mkdir(outPut.c_str());
            mkdir(check.c_str());
            return;
        }

        /*
        *   ��my_sfm_data�е�sence��תr��д��path�µ�ply�ļ�
        */
        bool check_matrix_right(const SfM_Data& my_sfm_data, Eigen::Matrix3d r, const string& path) {

            string header = "ply\nformat ascii 1.0\n";
            int number = my_sfm_data.GetPoses().size() + my_sfm_data.GetLandmarks().size();
            string header_1 = "element vertex " + std::to_string(number) + "\n";
            string header_2 = "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

            ofstream out(path);
            if (!out.is_open()) {
                std::cout << "Can't write check ply.\n";
                return false;
            }
            out << header + header_1 + header_2;
            auto & pose = my_sfm_data.GetPoses();
            for (auto &k : pose) {
                Eigen::Vector3d center = r * k.second.center();
                out << center(0) << ' ' << center(1) << ' ' << center(2) << ' ' << 0 << ' ' << 255 << ' ' << 0 << '\n';
            }
            auto structures = my_sfm_data.GetLandmarks();
            for (auto &k : structures) {
                Eigen::Vector3d point = r * k.second.X;
                out << point(0) << ' ' << point(1) << ' ' << point(2) << ' ' << 255 << ' ' << 255 << ' ' << 255 << '\n';
            }
            out << '\n';
            out.close();
            return true;
        }

        /*
        *   �������r��path
        */
        bool check_rotation_matrix(const string& path, Eigen::Matrix3d r) {
            std::ofstream out(path);
            if (!out.is_open()) {
                std::cout << "file open error in check_rotation_matrix\n";
                return false;
            }
            out << r;
            out.close();
            return true;
        }

        /*
        *   ���lab_data �� path
        *   ���� ���鵥λ��������
        */
        void Lab::Helper::check_labdata_rotations(const Lab::LabData& lab_data, const string& path) {
            std::ofstream out(path);
            for (const auto& e : lab_data.pose_sensor_rotation) {
                out << e.second << '\n';
            }
            for (const auto& e : lab_data.pose_sfm_rotation) {
                out << e.second << '\n';
            }
            out.close();
            return;
        }

        /*
        *  ���� ѡ��pose�����ȷ
        *  �����path��
        *  ���ݸ�ʽ
        *  ���㿪ʼ�ı�� ԭʼpose��� ԭʼͼ������
        */
        void Lab::Helper::check_labdata_choose_pose(const Lab::LabData& lab_data, const string& path) {
            std::ofstream out(path);
            if (!out.is_open()) {
                std::cerr << "file openning error in check_labdata_choose_pose\n";
                return;
            }
            for (const auto& e : lab_data.chosen_to_pose) {
                out << e <<  ' ';
                const auto iter = lab_data.pose_name.find(e);
                out << iter->second << std::endl;
            }
            out.close();
            return;
        }

        /*
        *  ���ֱ����ȡ�Ƿ���ȷ
        *  ��ԭͼ����ʾ��ѡ���ֱ��
        *  tempPath, ��Ž�ͼ��Ϣ���ļ���
        */
        void check_line_choose(const Lab::LabData &lab_data, const string &tempPath) {
            cv::namedWindow("check line", cv::WINDOW_NORMAL);
            for (const auto &e : lab_data.pose_line) {
                string imageName = lab_data.pose_name.at(e.first);
                cv::Mat image = cv::imread(lab_data.image_path + imageName);
                cv::line(image, e.second.getStartPoint(), e.second.getEndPoint(), { 0, 0, 255 }, 2);
                cv::imshow("check line", image);
                cv::waitKey();
            }
            return;
        }


        void check_line_point_3d(const vector<std::pair<cv::Point3d, cv::Point3d> >&pointLine, const string &path) {
            std::ofstream out(path);
            if (!out.is_open()) {
                std::cerr << "file openning error in check_line_point3d";
                return;
            }
            for (const auto &e : pointLine) {
                out << e.first << ' ' << e.second << '\n';
            }
            out.close();
            return;
        }
    }
}