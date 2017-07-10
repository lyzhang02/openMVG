#include "lab801_eric.hpp"

namespace Lab {
    /*
    @ overload
    @ �����Ա任����������
    @ input mapping 3*3 ���Ա任����
    @ input point 3d��
    @ output ���ú��Point3d
    */
    cv::Point3d mapPoint(const Eigen::Matrix<double, 3, 3> &mapping, cv::Point3d &point) {
        Eigen::Vector3d eigenPoint(point.x, point.y, point.z);
        Eigen::Vector3d a = mapping * eigenPoint;
        return cv::Point3d(a(0), a(1), a(2));
    }

    /*
    @ overload
    @ �������Ա任mapping�� pointLine�е�ÿһ����
    @ input mapping 3*3 �任����
    @ input&output pointLine ÿ��pair�е��������ʾ3dֱ�ߵ������յ�
    */
    void mapPoint(const Eigen::Matrix<double, 3, 3> &mapping, vector<pair<cv::Point3d, cv::Point3d> >&pointLine) {
        for (auto &e : pointLine) {
            auto a = mapPoint(mapping, e.first);
            e.first = a;
            a = mapPoint(mapping, e.second);
            e.second = a;
        }
        //std::cout << pointLine.size();
        return;
    }

    /*
    @ overload
    @ ���� v1�� v2, v3����ĸ����ǣ�
    @ ��[0,0,1]�ڻ���ģ������
    @ ���ؽǶ���
    */
    double angleA(double v1, double v2, double v3) {
        double t = v1*v1 + v2 * v2 + v3 * v3;
        if (t < 1e-6)
            return 0;
        double angle = v3 / std::sqrt(t);
        return std::acos(angle) * 180 / M_PI;
    }

    /*
    @ overload
    @ ��������3d����㸩����
    @ ����angleA(double, double, double)
    */
    double angleA(cv::Point3d up, cv::Point3d down) {
        return angleA(up.x - down.x, up.y - down.y, up.z - down.z);
    }

    /*
    @ overload
    @ ���ݶ�ά����(a,b)������y���˳ʱ��Ƕ�
    @ ���ؽǶ���
    */
    double angleB(double a, double b) {
        double x = 0, y = 1;
        double t = std::sqrt(a *a + b * b);
        t = b / t;
        t = std::acos(t) * 180 / M_PI;
        if (a < 0)
            t = 360 - t;
        return t;
    }

    /*
    @ overload
    @ ������3d�������y���˳ʱ��Ƕ�
    @ ����angle(double, double, double)
    */
    double angleB(cv::Point3d up, cv::Point3d down) {
        return angleB(up.x - down.x, up.y - down.y);
    }

    /*
    @ overload
    @ ����������3d������xyƽ���������ӵ�[a,b]������
    @ ����0�ȸ�������ƽ������
    */
    void angleB(double &a, double &b, cv::Point3d up, cv::Point3d down) {
        double t1 = up.x - down.x;
        double t2 = up.y - down.y;
        double t = t1 * t1 + t2 * t2;
        if (t < 1e-6)
            return;
        t = std::sqrt(t);
        t1 /= t;
        t2 /= t;
        a += t1;
        b += t2;
        return;
    }


    /*
    @ ���㸩���ǶȺ�ˮƽ�Ƕ�
    @ input pointLine ����vector<pair<Point3d, Point3d>> ÿ��pair�б�ʾ3d�ռ�ֱ�ߵ������˵�
    @ path ����Ƕȵ��ļ�����
    */
    void computeAngle(const vector<pair<cv::Point3d, cv::Point3d> >&pointLine, const string &path) {
        vector<double> angleH;
        vector<double> angleI;
        angleH.reserve(pointLine.size());
        angleI.reserve(pointLine.size());
        double vecA = 0, vecB = 0;
        for (const auto &e : pointLine) {
            if (e.first.z > e.second.z) {
                //double res = angleHorizon(e.first, e.second);
                angleH.push_back(angleA(e.first, e.second));
                angleI.push_back(angleB(e.first, e.second));
                angleB(vecA, vecB, e.first, e.second);
            }
            else {
                angleH.push_back(angleA(e.second, e.first));
                angleI.push_back(angleB(e.second, e.first));
                angleB(vecA, vecB, e.second, e.first);
            }
        }

        std::ofstream out(path);
        double sumH = 0;
        double sumI = 0;
        for (int i = 0; i < angleH.size(); ++i) {
            out << angleH[i] << '\t' << angleI[i] << '\n';
            sumH += angleH[i];
            sumI += angleI[i];
        }
        out << "$: " << sumH / angleH.size() << '\t' << sumI/angleI.size() << '\n';
        out << "&: " << angleB(vecA, vecB);
        out.close();
        return;
    }
}