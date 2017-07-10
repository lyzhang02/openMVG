#pragma once
#include <opencv2\line_descriptor.hpp>
#include <opencv2\core.hpp>
#include <vector>
#include "lab801_allen.hpp"
using cv::Mat;
using cv::line_descriptor::KeyLine;
namespace Lab {
    namespace UserInterface {
        void mark_line_one_image(const string &winName, const Mat &image, vector<cv::Point> &line);
        void detect_line_manually(vector<Mat> &images, vector<vector<KeyLine> > &lines);
    }
}