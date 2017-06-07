#include "lab801_diana.hpp"
#include "Eigen\StdVector"

using cv::Point2f;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::ComputeThinU;
using Eigen::ComputeThinV;
using Eigen::JacobiSVD;
namespace Lab {

    /*
    * 以groupSize个编号一组，将所有选定了直线的pose分组
    * 如 在1，2，4，5，6号pose上选择的有直线，三个一组生成 5- 3 + 1 = 3组测量单位
    * {1，2，4}，{2，4，5}，{4，5，6}
    */
    void generateGroup(const LabData &lab_data, int groupSize, vector<vector<int>> &group) {
        int lineNum = lab_data.pose_line.size();
        groupSize = groupSize > lineNum ? lineNum : groupSize;
        int groupNum = lineNum - groupSize + 1;
        group = vector<vector<int>>(groupNum, vector<int>(groupSize, 0));
        vector<int> originPoses; //记录直线所在的原始pose，便于进行排列分组
        originPoses.reserve(lineNum);
        for (const auto &e : lab_data.pose_line) {
            originPoses.push_back(e.first);
        }
        for (int i = 0; i < group.size(); ++i) {
            for (int j = 0; j < groupSize; ++j) {
                group[i][j] = originPoses[i + j];
            }
        }
        return;
    }


    void pluckerCorrection(Matrix<double, 6, 1> &pluckerLine) {
        Eigen::MatrixXd ab(3,2);
        ab << pluckerLine(0), pluckerLine(3),
            pluckerLine(1), pluckerLine(4),
            pluckerLine(2), pluckerLine(5);
        JacobiSVD<MatrixXd > svdAB(ab, ComputeThinU | ComputeThinV);
        Eigen::Matrix2d diagEigenValue = Eigen::Matrix2d::Zero();
        for (int i = 0; i < 2; ++i) {
            diagEigenValue(i, i) = svdAB.singularValues()(i);
        }
        Eigen::Matrix2d Z = diagEigenValue * svdAB.matrixV().transpose();
        Eigen::Matrix2d T;
        T << Z(1, 0), Z(1, 1),
            Z(0, 1), -Z(0, 0);
        JacobiSVD<Eigen::Matrix2d> svdT(T, ComputeThinU | ComputeThinV);
        Matrix<double, 2, 1> minVector = svdT.matrixV().block<2, 1>(0, 1);
        Eigen::Matrix2d VFormedByT;
        VFormedByT << minVector(0), -minVector(1),
            minVector(1), minVector(0);
        Eigen::Matrix2d tempDiag = VFormedByT * diagEigenValue * svdAB.matrixV().transpose();
        tempDiag(0, 1) = 0;
        tempDiag(1, 0) = 0;
        Matrix<double, 3, 2> UV = svdAB.matrixU() * VFormedByT * tempDiag;
        pluckerLine << UV.block<3, 1>(0, 0), UV.block<3, 1>(0, 1);
        return;

    }

    //!!此处没有将直线的两个端点按照vector<pair<Vector2d,Vector2d>> 保存，因为不确定是否需要指定allocator
    Matrix<double, 6, 1> reconstructure_from_group_line(vectorMat34d &POneGroup, vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &lineOneGroup) {
        int n = POneGroup.size();
        MatrixXd A(2 * n, 6); 
        //vector<Eigen::Matrix<double, 2, 6> , Eigen::aligned_allocator<Eigen::Matrix<double, 2, 6> > > rowForA;
        //rowForA.reserve(n);
        for (int i = 0; i < POneGroup.size(); ++i) {
            Matrix<double, 2, 3> twoPoint; // [x | y]T
            twoPoint << lineOneGroup[2 * i](0), lineOneGroup[2 * i](1), 1,
                lineOneGroup[2 * i + 1](0), lineOneGroup[2 * i + 1](1), 1;
            Matrix<double, 3, 3> PColumn3;
            PColumn3 << POneGroup[i](0, 0), POneGroup[i](0, 1), POneGroup[i](0, 2),
                POneGroup[i](1, 0), POneGroup[i](1, 1), POneGroup[i](1, 2),
                POneGroup[i](2, 0), POneGroup[i](2, 1), POneGroup[i](2, 2);
            Matrix<double, 3, 3> PColumnCross4;
            PColumnCross4 << 0, -POneGroup[i](2, 3), POneGroup[i](1, 3),
                POneGroup[i](2, 3), 0, -POneGroup[i](0, 3),
                -POneGroup[i](1, 3), POneGroup[i](0, 3), 0;
            Matrix<double, 3, 6> PforLine; //det(P[1:3])P[1:3]-T | [p[4]]x P[1:3]
            PforLine << PColumn3.determinant() * PColumn3.transpose().inverse(), PColumnCross4 * PColumn3;
            //rowForA.push_back(twoPoint * PforLine);
            A.block<2, 6>(2 * i, 0) = twoPoint * PforLine;
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svdSolver(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Matrix<double, 6, 1> minEigenVector;
        minEigenVector << svdSolver.matrixV().block<6, 1>(0, 5);
        return minEigenVector;
        
    }

    void reconstruction_line_Linear(const SfM_Data &sfm_data, const LabData &lab_data, int groupSize, 
        vectorVec61d &outPluckerLine) {
        if (groupSize < 2)
            return;
        vector<vector<int>> group;
        generateGroup(lab_data, groupSize, group);
        outPluckerLine.reserve(group.size());
        for (const auto &e : group) {
            vectorMat34d POneGroup;
            POneGroup.reserve(groupSize); //每组pose的camera matrix
            vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >  lineOneGroup; //每组pose选定的line
            lineOneGroup.reserve(groupSize * 2);
            for (const auto &k : e) {
                POneGroup.push_back(lab_data.pose_P.at(k)); // 一个分组内的groupSize个 camera matrix

                //一个分组内groupSize条直线的起点和终点
                Point2f cvStartPoint= lab_data.pose_line.at(k).getStartPoint();
                Point2f cvEndPoint = lab_data.pose_line.at(k).getEndPoint();
                Eigen::Vector2d startPoint{ cvStartPoint.x, cvStartPoint.y };
                Eigen::Vector2d endPoint{ cvEndPoint.x, cvEndPoint.y };
                const auto &view = sfm_data.views.at(k);  // view k
                auto instrinsicSharedPtr = sfm_data.intrinsics.at(view->id_view);
                auto undistortStartPoint = instrinsicSharedPtr->get_ud_pixel(startPoint); // undistort point
                auto undistortEndPoint = instrinsicSharedPtr->get_ud_pixel(endPoint);  // undistort point
                lineOneGroup.push_back(undistortStartPoint);
                lineOneGroup.push_back(undistortEndPoint);
            }
            Matrix<double, 6,1 > pluckerLine = reconstructure_from_group_line(POneGroup, lineOneGroup);
            pluckerCorrection(pluckerLine);
            outPluckerLine.push_back(pluckerLine);
        }
        return;
    }

    void plucker2Point(const vectorVec61d &pluckerLine, vector<pair<cv::Point3d, cv::Point3d> >& outPointLine) {
        for (const auto &e : pluckerLine) {
            cv::Point3d point1{ -e(1) / e(5), e(0) / e(5), 0};
            cv::Point3d point2{ (e(3) - e(1))/e(5), (e(4) + e(0)) / e(5), 1};
            outPointLine.push_back(std::make_pair(point1, point2));
        }
    }
}