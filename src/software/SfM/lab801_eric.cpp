#include "lab801_eric.hpp"

namespace Lab {
    /*
    @ overload
    @ 将线性变换作用于向量
    @ input mapping 3*3 线性变换矩阵
    @ input point 3d点
    @ output 作用后的Point3d
    */
    cv::Point3d mapPoint(const Eigen::Matrix<double, 3, 3> &mapping, cv::Point3d &point) {
        Eigen::Vector3d eigenPoint(point.x, point.y, point.z);
        Eigen::Vector3d a = mapping * eigenPoint;
        return cv::Point3d(a(0), a(1), a(2));
    }

    /*
    @ overload
    @ 作用线性变换mapping于 pointLine中的每一个点
    @ input mapping 3*3 变换矩阵
    @ input&output pointLine 每个pair中的两个点表示3d直线的起点和终点
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
    @ 向量 v1， v2, v3代表的俯仰角，
    @ 与[0,0,1]内积除模反余弦
    @ 返回角度制
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
    @ 根据两个3d点计算俯仰角
    @ 调用angleA(double, double, double)
    */
    double angleA(cv::Point3d up, cv::Point3d down) {
        return angleA(up.x - down.x, up.y - down.y, up.z - down.z);
    }

    /*
    @ overload
    @ 根据二维向量(a,b)计算与y轴的顺时针角度
    @ 返回角度制
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
    @ 由两个3d点计算与y轴的顺时针角度
    @ 调用angle(double, double, double)
    */
    double angleB(cv::Point3d up, cv::Point3d down) {
        return angleB(up.x - down.x, up.y - down.y);
    }

    /*
    @ overload
    @ 由上下两个3d点计算出xy平面向量，加到[a,b]向量上
    @ 避免0度附近算术平均错误
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
    @ 计算俯仰角度和水平角度
    @ input pointLine 类型vector<pair<Point3d, Point3d>> 每个pair中表示3d空间直线的两个端点
    @ path 输出角度的文件名称
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