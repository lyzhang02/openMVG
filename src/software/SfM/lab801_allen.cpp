#include "lab801_allen.hpp"

using namespace openMVG::sfm;
using namespace std;

namespace Lab{
    Lab::LabData Lab::createLabData(const SfM_Data& sfm_data) {
        /*
        *��ȡsfm_data��¼ �ؽ��ɹ��ı�ţ����Ŷ�Ӧ����ת�� ͼ������
        */
        Lab::LabData res;
        res.validPose = sfm_data.poses.size();
        res.image_path = sfm_data.s_root_path + "/";
        res.root_path = res.image_path + "/../";
        //��¼��pose�ı�ţ���Ӧ����ת���󣬶�Ӧ��ͼƬ����
        for (Poses::const_iterator iter = sfm_data.poses.cbegin(); iter != sfm_data.poses.cend(); ++iter) {
            res.pose_sfm_rotation.insert({ iter->first, iter->second.rotation() });
            res.pose_name.insert({ iter->first, sfm_data.views.at(iter->first)->s_Img_path });
        }
        if (res.pose_name.size() < 3)
            exit(1);
        return res;
    }

    /*
    *   �����ȡ�ļ�fileName�ڵ���Ϣ
    *   ���룺 fileName�� �ļ�����
    *           tag1, tag2, ��ʶ��Ϣ��ʼ���ַ���
    *   ����� rotation ��ȡ���ľ���
    *           GPS ��ȡ����GPS
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
    *   ����ͼƬ��ص�txt�ڶ�ȡ������������Ϣ
    *   ���룺 filePath��txt���ڵ�Ŀ¼
    *           tag1 tag2�� txt�ڱ�ʶ��Ϣ��ʼ���ַ���
    *   ����� lab_data�� ��Ϣд��lab_data�ڵ�pose_sensor_rotation �� pose_gps������ϣmap
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
    *   ����λ��������ת��Ϊ�����Ƕȣ� �ο�����https://www.learnopencv.com/rotation-matrix-to-euler-angles/
    *   ���룺 r һ��������ת�ĵ�λ��������
    *   ����� Vector3f 3Ԫ���������ֱ������x,y,z��ĽǶȣ������Ʊ�ʾ
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
    *   �������ǶȺϳɵ�λ�������󣬰���˳��Z-Y-X(yaw, pitch, roll)
    *   ���룺 vec��Ԫ���������ֱ����x,y,z��ת�Ƕȣ����ȱ�ʾ
    *   ����� �ϳɵĵ�λ��������
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
    * ����ת�������ƽ�����㣬��ƽ��ָ�򵥵�����ƽ��
    * ���룺������ת����matrix_r�����о���Ӧ�ñȽ���� outFile�����Ϊ�գ���ô���ֽ��Ƕ�д��outFile���ļ���
    * �����ƽ�������ת����
    * �����������о�����зֽ��������eular�Ƕ�
    *       ���Ƕ�ƽ��֮���ٺϳɵ�λ��������
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
    * ���������ĳ�������ͼ��Pose�ĳ��������������
    * ���룺lab_data  �����õ������ݵĶ��� pose_sfm_rotations & pose_sensor_rotation
    * �����Matrix3d 3*3 ����
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
    * �ռ� ѡ��ֱ�ߵ�pose��P������Ϣ 
    */

    void read_P(LabData &lab_data, const SfM_Data &sfm_data) {

        for (const auto &e : lab_data.pose_line) {
            //std::shared_ptr<openMVG::sfm::View> view = sfm_data.GetViews().at(originPose);
            std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic = sfm_data.GetIntrinsics().at(e.first);
            //openMVG::geometry::Pose3 pose = sfm_data.GetPoseOrDie(view.get());
            openMVG::geometry::Pose3 pose = sfm_data.poses.at(e.first);
            openMVG::Mat34 p = intrinsic->get_projective_equivalent(pose);
            lab_data.pose_P.insert({ e.first, p}); //����pose��ţ���Ӧ��camera matrix
        }
    }
}