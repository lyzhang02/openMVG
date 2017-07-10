#include "lab801_crystal.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\imgproc.hpp"
#include <queue>
namespace Lab {
    namespace UserInterface {

        /*
        *   选择需要的pose号，并按照pose号从小到大的顺序保存在LabData的chosen_to_pose中
        */
        bool choose_pose(Lab::LabData& lab_data) {
            std::cout << "有效pose号:\n";
            for (const auto& e : lab_data.pose_sfm_rotation) {
                std::cout << e.first << ' ';
            }
            std::cout << std::endl;
            std::cout << "需要的pose数量: ";
            int n = 0;
            std::cin >> n;
            if (n < 3)
                return false;
            //std::vector<int> pose_chosen;
            //pose_chosen.reserve(n);
            int c_pose = 0;
            for (int i = 0; i < n; ++i) {
                std::cin >> c_pose;
                //pose_chosen.push_back(c_pose);
                lab_data.chosen_to_pose.insert(c_pose);
            }
            //std::sort(pose_chosen.begin(), pose_chosen.end());
            //return initial_chosen_to_pose(lab_data, pose_chosen);
            return true;
        }


        /*
        *   将选择的pose号按照从小到大的顺序保存在labdata中
        */
        bool initial_chosen_to_pose(Lab::LabData& lab_data, const vector<int>& pose_chosen, bool fileOrMemory, const string& fileName) {
            if (!fileOrMemory && pose_chosen.empty()) {
                return false;
            }
            if (!fileOrMemory) {
                for (int i = 0; i < pose_chosen.size(); ++i) {
                    lab_data.chosen_to_pose.insert({ i, pose_chosen[i]});
                }
                return true;
            }
            return false;
        }


        //bool gflag = false;
        
        struct RectangleCornor {
            //截图信息，作为userData传递给回掉函数
            bool flag;      //标识是否处于截图中，鼠标左键按下
            cv::Mat image;          //被截图的图像
            cv::Point startPoint;   //左上角
            cv::Point endPoint;     //右下角
            string winName;         //显示的窗口名称
        };

        /*
        * 截图鼠标回掉函数
        * userData 存储截图信息的用户结构 ,见上RectangleCornor
        * event 事件代码
        * x, y 当前鼠标的位置
        */
        void onMouseCallBack(int event, int x, int y, int, void* userData) {
            RectangleCornor* rec = (RectangleCornor*)userData;
            if (!rec->flag && event == cv::EVENT_LBUTTONDOWN) { //鼠标左键按下，记录此时的鼠标位置作为左上角点
                rec->flag = true;
                rec->startPoint.x = x;
                rec->startPoint.y = y;
                std::cout << "Down\n";
                return;
            }
            if (rec->flag && event == cv::EVENT_LBUTTONUP) { //鼠标左键弹开，记录此时的鼠标位置作为右下角点
                std::cout << "UP\n";
                cv::Rect r = cv::Rect(rec->startPoint, cv::Point{ x,y });
                cv::Rect full = cv::Rect({ 0,0 }, rec->image.size());
                r &= full;
                rec->endPoint.x = r.x + r.width;
                rec->endPoint.y = r.y + r.height;
                cv::Mat show = rec->image.clone();
                cv::rectangle(show, rec->startPoint, rec->endPoint, cv::Scalar{ 0, 255, 0 }, 2);
                cv::imshow(rec->winName, show);
                rec->flag = false;
                return;
            }
            return;
        }

        /*
        * 截图
        * 输入 winName， 显示图片的窗口名称
        * 输入 picPath， 输入图像的目录
        * 输入 pic，     输入图像的名称， picPath + pic 组成图片的绝对路径
        * 输入 outPutPath， 输出截取图像的目录，
        * outPutPath + pic.txt 组成记录截图在原图中左上角位置和宽高的数字
        * outPutPath + pic.jpg 组成截图结果的绝对路径
        */
        bool capture_picture(const string& winName, const string& picPath, const string& pic, const string& outPutPath) {
            const string picName = picPath + pic;   //被截图的图片绝对路径
            cv::Mat image = cv::imread(picName);
            if (image.rows < 1 || image.cols < 1) {
                std::cerr << "Picture read error in capture_picture.\n";
                return false;
            }
            cv::imshow(winName, image);
            RectangleCornor recInfo{ false, image, {0,0},{0,0}, winName};   //此次截图的信息结构
            cv::setMouseCallback(winName, onMouseCallBack, &recInfo);       //截图过程
            cv::waitKey();
            cv::Rect r{ recInfo.startPoint, recInfo.endPoint }; 
            std::string outName = outPutPath + pic;
            std::ofstream out(outName+".txt");
            if (!out.is_open()) {
                std:cerr << "file opening error in capture_picture.\n";
                return false;
            }
            out << r.x << ' ' << r.y << ' ' << r.width << ' ' << r.height;
            out.close();
            cv::Mat cut = cv::Mat(image, r);
            cv::imwrite(outName + ".jpg", cut);
            return true;
        }

        /*
        * 直线选择，显示绘制有直线的截图
        * 选择所需编号, 保存在labData中的pose_line_zero
        */
        void choose_line(const vector<Mat> &images, const vector<vector<KeyLine>> &lines,  LabData &labData) {
            for (int i = 0; i < images.size(); ++i) {
                string winName = std::to_string(i);
                cv::namedWindow(winName, cv::WINDOW_NORMAL);
                cv::imshow(winName, images[i]);
                cv::waitKey(0);
            }

            std::cout << "选择直线:\n";
            int i = 0;
            for (const auto &e : labData.chosen_to_pose) {
                int n = 0;
                int lineSize = lines[i].size();
                std::cin >> n;
                if (n > 0) {
                    n = (n > lineSize) ? lineSize : n;
                    labData.pose_line[e] = lines[i][lineSize - n];
                }
                ++i;
            }
            /*for (int i = 0; i < images.size(); ++i) {
                int n = 0;
                int lineSize = lines[i].size();
                std::cin >> n;
                if (n > 0) {
                    n = (n > lineSize) ? lineSize : n;
                    labData.pose_line_zero[i] = lines[i][lineSize - n];
                }
            }*/
            if (labData.pose_line.size() < 3) {
                std::cerr << "less than 3, error in choose_line.\n";
            }
            //cv::destroyAllWindows();
            return;
        }

        void choose_line_manually(const vector<Mat> &images, const vector<vector<KeyLine>> &lines, LabData &labData) {
            for (int i = 0; i < images.size(); ++i) {
                string winName = std::to_string(i);
                cv::namedWindow(winName, cv::WINDOW_NORMAL);
                cv::imshow(winName, images[i]);
                cv::waitKey(0);
            }

            std::cout << "选择直线:\n";
            int i = 0;
            for (const auto &e : labData.chosen_to_pose) {
                int n = 0;
                int lineSize = lines[i].size();
                std::cin >> n;
                if (n > 0) {
                    n = (n > lineSize) ? lineSize : n;
                    labData.pose_line[e] = lines[i][n];
                }
                ++i;
            }
            /*for (int i = 0; i < images.size(); ++i) {
            int n = 0;
            int lineSize = lines[i].size();
            std::cin >> n;
            if (n > 0) {
            n = (n > lineSize) ? lineSize : n;
            labData.pose_line_zero[i] = lines[i][lineSize - n];
            }
            }*/
            if (labData.pose_line.size() < 3) {
                std::cerr << "less than 3, error in choose_line.\n";
            }
            //cv::destroyAllWindows();
            return;
        }

    } //! UserInterface


    /*
    * 函数类，比较两个KeyLine代表的直线长度大小  > (greater)顺序
    */
    class LineCompare {
    private:
        float distance(float startX, float startY, float endX, float endY) {
            float axisX = endX - startX;
            float axisY = endY - startY;
            return axisX * axisX + axisY * axisY;
        }
    public:
        bool operator()(const KeyLine& line1, const KeyLine& line2) {
            float length1 = distance(line1.startPointX, line1.startPointY, line1.endPointX, line1.endPointY);
            float length2 = distance(line2.startPointX, line2.startPointY, line2.endPointX, line2.endPointY);
            return length1 > length2;
        }
    };


    using cv::line_descriptor::KeyLine;
    /*
    *   利用最小堆排序出前longK个最长的直线
    *   输入 allLine， 一幅图像中提取出的所有直线
    *   输入 longK， 最终留下的直线数量
    *   输出， outLine， 筛选出的前longK长的直线 outline.back()最长
    */
    void topKLines(const vector<KeyLine>& allLine, vector<KeyLine>& outLine, int longK) {
        std::priority_queue<KeyLine, vector<KeyLine>, LineCompare> longLine;
        int i = 0;
        int temp = (allLine.size() <= longK) ? allLine.size() : longK;
        for (i = 0; i < temp; ++i) {
            longLine.push(allLine[i]);
        }
        while (i < allLine.size()) {
            longLine.push(allLine[i]);
            longLine.pop();
            ++i;
        }
        
        outLine.reserve(longLine.size());
        while (!longLine.empty()) {
            outLine.push_back(longLine.top());
            longLine.pop();
        }
        return;
    }

    /*
    *   检测直线，并选择出前longK条直线
    *   输入 images，图像集合
    *   输入 longK， 每张图象返回的直线数量
    *   输出 lines， 直线集合，每个元素对应一幅图像中的前longK条直线
    */
    void detect_line(std::vector<cv::Mat>& images, std::vector<std::vector<KeyLine>>& lines,int longK) {
        using cv::Ptr;
        using cv::line_descriptor::LSDDetector;
        // lsd直线检测器,同时针对images数组中的每一张图进行检测
        vector<vector<KeyLine>> allLines;
        allLines.reserve(images.size());

        Ptr<LSDDetector> lineDetector = LSDDetector::createLSDDetector();
        vector<KeyLine> temp;
        for (const auto &e : images) {
            lineDetector->detect(e, temp, 2, 3);
            allLines.push_back(std::move(temp));
        }
        
        temp.clear();
        for (const auto& k : allLines) {
            topKLines(k, temp, longK); // 比较出longK个最长直线保存在temp中
            lines.push_back(std::move(temp));
        }
        return;
    }



    class ColorTable {//预先声明的颜色表
    public:
        const static int table_size = 5;
        cv::Scalar color[table_size]{ { 0,0,255 },{ 255,0,255 },{ 0,255,0 },{ 255,255,0 },{ 0,255,255 } };
    };

    /* 在图像上绘制直线
    *  输入 images 图像集合
    *  输入 lines 直线集合
    *  imges和lines对应
    */
    void draw_lines(vector<Mat>& images, vector<vector<KeyLine>>& lines) {
        for (int i = 0; i < images.size(); ++i) {
            //Mat dst;
            cv::line_descriptor::drawKeylines(images[i], lines[i], images[i]);
            cv::imshow("line", images[i]);
            cv::waitKey();
        }
        return;
    }


    /*
    *  在图像上绘制直线 针对单幅图片
    *  输入 image 图像
    *  输入 需要绘制的直线
    */
    void draw_lines(const Mat &image, const vector<KeyLine> &lines) {
        ColorTable color;
        for (int i = 0; i < lines.size(); ++i) {
            cv::Point pt1{ lines[i].getStartPoint() };
            cv::Point pt2{ lines[i].getEndPoint() };
            cv::line(image, pt1, pt2, color.color[i % ColorTable::table_size], 2);
            //cv::line(image, pt1, pt2, { 0, 0, 255 }, 2);
            cv::putText(image, std::to_string(lines.size() - i), (pt1 + pt2) / 2, cv::FONT_HERSHEY_SIMPLEX, 1, color.color[i % ColorTable::table_size], 2);
        }
    }

    void line_origin_image(Lab::LabData &lab_data, const string &tempPath) {
        for (auto &originPose_line : lab_data.pose_line) {
            //int originPose = lab_data.chosen_to_pose.at(e.first);
            string imageName = lab_data.pose_name.at(originPose_line.first);
            std::ifstream read_in(tempPath + imageName + ".txt");
            if (!read_in.is_open()) {
                std::cerr << "txt open error in check_lin_choose.\n";
                return;
            }
            cv::Point2f leftUp;
            read_in >> leftUp.x >> leftUp.y;
            read_in.close();
            originPose_line.second.startPointX += leftUp.x;
            originPose_line.second.startPointY += leftUp.y;
            originPose_line.second.endPointX += leftUp.x;
            originPose_line.second.endPointY += leftUp.y;
        }
        return;
    }
}