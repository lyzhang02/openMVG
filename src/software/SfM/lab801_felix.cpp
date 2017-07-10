#include "lab801_felix.hpp"

namespace Lab {

    void onMouseCallBackMarkPoint(int event, int x, int y, int, void *userData) {
        vector<cv::Point> *ptr_line = (vector<cv::Point> *) userData;
        if (event == cv::EVENT_LBUTTONUP) {
            ptr_line->push_back(cv::Point{ x, y });
            std::cout << "U\n";
        }
        return;
    }
    void UserInterface::mark_line_one_image(const string &winName, const Mat &image, vector<cv::Point> &line) {
        cv::imshow(winName, image);
        cv::setMouseCallback(winName, onMouseCallBackMarkPoint, &line);
        cv::waitKey();
        return;
    }



    void UserInterface::detect_line_manually(vector<Mat> &images, vector<vector<KeyLine>> &lines) {
        lines.reserve(images.size());
        const string winName = "mark";
        cv::namedWindow(winName, cv::WINDOW_NORMAL);
        for (const auto &e : images) {
            vector<cv::Point> point_temp;
            mark_line_one_image(winName, e, point_temp);
            if (point_temp.size() % 2 != 0) {
                lines.push_back(vector<KeyLine>{});
            }
            else {
                vector<KeyLine> keyLine_temp;
                for (int i = 0; i * 2 < point_temp.size(); ++i) {
                    KeyLine t;
                    t.startPointX = point_temp[2 * i].x;
                    t.startPointY = point_temp[2 * i].y;
                    t.endPointX = point_temp[2 * i + 1].x;
                    t.endPointY = point_temp[2 * i + 1].y;
                    keyLine_temp.push_back(t);
                }
                //std::cout << keyLine_temp.size() << '\n';
                lines.push_back(std::move(keyLine_temp));

            }
        }
        return;
    }
}