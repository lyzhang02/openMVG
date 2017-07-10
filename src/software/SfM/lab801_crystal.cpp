#include "lab801_crystal.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\imgproc.hpp"
#include <queue>
namespace Lab {
    namespace UserInterface {

        /*
        *   ѡ����Ҫ��pose�ţ�������pose�Ŵ�С�����˳�򱣴���LabData��chosen_to_pose��
        */
        bool choose_pose(Lab::LabData& lab_data) {
            std::cout << "��Чpose��:\n";
            for (const auto& e : lab_data.pose_sfm_rotation) {
                std::cout << e.first << ' ';
            }
            std::cout << std::endl;
            std::cout << "��Ҫ��pose����: ";
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
        *   ��ѡ���pose�Ű��մ�С�����˳�򱣴���labdata��
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
            //��ͼ��Ϣ����ΪuserData���ݸ��ص�����
            bool flag;      //��ʶ�Ƿ��ڽ�ͼ�У�����������
            cv::Mat image;          //����ͼ��ͼ��
            cv::Point startPoint;   //���Ͻ�
            cv::Point endPoint;     //���½�
            string winName;         //��ʾ�Ĵ�������
        };

        /*
        * ��ͼ���ص�����
        * userData �洢��ͼ��Ϣ���û��ṹ ,����RectangleCornor
        * event �¼�����
        * x, y ��ǰ����λ��
        */
        void onMouseCallBack(int event, int x, int y, int, void* userData) {
            RectangleCornor* rec = (RectangleCornor*)userData;
            if (!rec->flag && event == cv::EVENT_LBUTTONDOWN) { //���������£���¼��ʱ�����λ����Ϊ���Ͻǵ�
                rec->flag = true;
                rec->startPoint.x = x;
                rec->startPoint.y = y;
                std::cout << "Down\n";
                return;
            }
            if (rec->flag && event == cv::EVENT_LBUTTONUP) { //��������������¼��ʱ�����λ����Ϊ���½ǵ�
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
        * ��ͼ
        * ���� winName�� ��ʾͼƬ�Ĵ�������
        * ���� picPath�� ����ͼ���Ŀ¼
        * ���� pic��     ����ͼ������ƣ� picPath + pic ���ͼƬ�ľ���·��
        * ���� outPutPath�� �����ȡͼ���Ŀ¼��
        * outPutPath + pic.txt ��ɼ�¼��ͼ��ԭͼ�����Ͻ�λ�úͿ�ߵ�����
        * outPutPath + pic.jpg ��ɽ�ͼ����ľ���·��
        */
        bool capture_picture(const string& winName, const string& picPath, const string& pic, const string& outPutPath) {
            const string picName = picPath + pic;   //����ͼ��ͼƬ����·��
            cv::Mat image = cv::imread(picName);
            if (image.rows < 1 || image.cols < 1) {
                std::cerr << "Picture read error in capture_picture.\n";
                return false;
            }
            cv::imshow(winName, image);
            RectangleCornor recInfo{ false, image, {0,0},{0,0}, winName};   //�˴ν�ͼ����Ϣ�ṹ
            cv::setMouseCallback(winName, onMouseCallBack, &recInfo);       //��ͼ����
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
        * ֱ��ѡ����ʾ������ֱ�ߵĽ�ͼ
        * ѡ��������, ������labData�е�pose_line_zero
        */
        void choose_line(const vector<Mat> &images, const vector<vector<KeyLine>> &lines,  LabData &labData) {
            for (int i = 0; i < images.size(); ++i) {
                string winName = std::to_string(i);
                cv::namedWindow(winName, cv::WINDOW_NORMAL);
                cv::imshow(winName, images[i]);
                cv::waitKey(0);
            }

            std::cout << "ѡ��ֱ��:\n";
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

            std::cout << "ѡ��ֱ��:\n";
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
    * �����࣬�Ƚ�����KeyLine�����ֱ�߳��ȴ�С  > (greater)˳��
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
    *   ������С�������ǰlongK�����ֱ��
    *   ���� allLine�� һ��ͼ������ȡ��������ֱ��
    *   ���� longK�� �������µ�ֱ������
    *   ����� outLine�� ɸѡ����ǰlongK����ֱ�� outline.back()�
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
    *   ���ֱ�ߣ���ѡ���ǰlongK��ֱ��
    *   ���� images��ͼ�񼯺�
    *   ���� longK�� ÿ��ͼ�󷵻ص�ֱ������
    *   ��� lines�� ֱ�߼��ϣ�ÿ��Ԫ�ض�Ӧһ��ͼ���е�ǰlongK��ֱ��
    */
    void detect_line(std::vector<cv::Mat>& images, std::vector<std::vector<KeyLine>>& lines,int longK) {
        using cv::Ptr;
        using cv::line_descriptor::LSDDetector;
        // lsdֱ�߼����,ͬʱ���images�����е�ÿһ��ͼ���м��
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
            topKLines(k, temp, longK); // �Ƚϳ�longK���ֱ�߱�����temp��
            lines.push_back(std::move(temp));
        }
        return;
    }



    class ColorTable {//Ԥ����������ɫ��
    public:
        const static int table_size = 5;
        cv::Scalar color[table_size]{ { 0,0,255 },{ 255,0,255 },{ 0,255,0 },{ 255,255,0 },{ 0,255,255 } };
    };

    /* ��ͼ���ϻ���ֱ��
    *  ���� images ͼ�񼯺�
    *  ���� lines ֱ�߼���
    *  imges��lines��Ӧ
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
    *  ��ͼ���ϻ���ֱ�� ��Ե���ͼƬ
    *  ���� image ͼ��
    *  ���� ��Ҫ���Ƶ�ֱ��
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