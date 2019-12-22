#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/contrib/contrib.hpp>//Directory
#include <stdio.h>
#include <iostream>
// #include "popt_pp.h"
#include <sys/stat.h>

using namespace std;
using namespace cv;
                                       
int width = 8;
int height = 6;

class Laneline_Detection{
public:
    void read_images(){
        // cv::Directory dir;//opencv 类
        // fileNames = dir.GetListFiles(dir_path,"*.jpg", false);
        cv:String jpg = "/media/vickylzy/文件共享盘/A3_grade_one/class/computer_vision/Assignment/Assignment3/png";
        // std::vector<cv::String> fileNames;//名字列表
        cv::glob(jpg, fileNames);
    }

    bool cannyedge(const cv::String& filename){
        imageInput = cv::imread(filename);
        Mat imageinput(imageInput, Rect(0,60,imageInput.cols,imageInput.rows-60));//截掉视频上方水印
        if(!imageInput.data){
            cout << "error in loading picture" << endl;
            return false;
        }
        Mat DstPic;
        cv::cvtColor(imageinput, imagesrc, CV_RGB2GRAY);//转灰度图
        GaussianBlur(imagesrc, image, Size(9,9), 0);//高斯模糊
        Canny(image, edge, 75, 150);//计算梯度、非极大值抑制、双阈值
        // DstPic.create(imageInput.size(),imageInput.type());
        // DstPic = Scalar::all(0);//目标图
        // imageInput.copyTo(DstPic, edge);//使用Canny算子输出的边缘图edge作为掩码，来将原图拷到目标图Dst中

        // namedWindow("input", CV_WINDOW_AUTOSIZE);
        // cv::imshow("edge", edge);
        // waitKey(0);
        return true;
    }

    void lane_detection(){
        // vector<Vec2f> lines;//定义一个存放直线信息的向量//Hough直线检测API
        // HoughLines(Laneline_Detection::edge, lines, 1, CV_PI / 180, 70);
        // for(size_t i = 0;i < lines.size(); i++){
        //     float rho = lines[i][0], theta = lines[i][1];
        //     Point pt1, pt2;
        //     double a = cos(theta), b = sin(theta);
        //     double x0 = a*rho, y0 = b*rho;
        //     pt1.x = cvRound(x0 + 1000 * (-b));
        //     pt1.y = cvRound(y0 + 1000 * (a));
        //     pt2.x = cvRound(x0 - 1000 * (-b));
        //     pt2.y = cvRound(y0 - 1000 * (a));
        //     line(imagesrc, pt1, pt2, Scalar(55, 255, 0), 2, LINE_AA);
        // }        
        vector<Vec4f> plines;
        HoughLinesP(Laneline_Detection::edge, plines, 1, CV_PI / 180, 70, 100, 7);
        //InputArray src, // 输入图像，必须8-bit的灰度图像；OutputArray lines, // 输出的极坐标来表示直线；double rho, // 生成极坐标时候的像素扫描步长；double theta, //生成极坐标时候的角度步长，一般取值CV_PI/180；int threshold, // 阈值，只有获得足够交点的极坐标点才被看成是直线；double minLineLength = 0;// 最小直线长度；double maxLineGap = 0;// 最大间隔
        //共享参数
        //标记出直线
        vector<double> Theta;
        for (size_t i = 0; i < plines.size(); i++)
        {
            Vec4f point1 = plines[i];
            double theta;
            if (point1[2] != point1[0]){
                theta = atan2((point1[3] - point1[1]), (point1[2] - point1[0]));
            }
            else{
                theta = 90;

            }
            Theta.push_back(theta);
            line(imagesrc, Point(point1[0], point1[1]), Point(point1[2], point1[3]), Scalar(255, 255, 0), 2, LINE_AA);
        }        
        imshow("hough", imagesrc);
        waitKey(0);
        sort(Theta.begin(), Theta.end());
        for (size_t i = 0; i < Theta.size(); i++){
            cout << Theta[i] << ' ';
        }

    }
    std::vector<cv::String> fileNames;
private:
    int width = 8;
    int height = 6;
    int image_count = 0;
    bool found = false;
    Size winSize = cv::Size(5,5);
    Size zeroZone= cv::Size(-1,-1);
    Size patternSize = cv::Size(width, height);/* 每行、列的角点数 */
    Size square_size = Size(5,5);  /* 实际测量得到的标定板上每个棋盘格的大小 */
    Size image_size;
    TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01);//精度到0.01或者次数到30，亚像素角点检测算法便退出;
    vector<Point2f> image_points_buf;//一张图片经过角点提取，得到的所有角点坐标
    vector<vector<Point2f>> image_points;//所有图像的角点坐标
    vector<Point2f> imgp_projection;//重计算的某张图像的角点坐标
    vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
    // /*内外参数*/
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */
    Mat newMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    // Mat cameraMatrix = (Mat_<double>(3,3) << 1497.40903, 0, 1296.60132,0, 1485.36314, 1002.99673,0, 0, 1);
    // Mat distCoeffs = (Mat_<double>(1,5) <<-0.323753008, 0.785988860, 0.0013986356, -0.00132493339, -2.26195715);
    vector<Mat> tvecsMat;  /* 图像的平移向量 */
    vector<Mat> rvecsMat; /* 图像的旋转向量 */
    vector<int> point_counts;  // 图像中角点的数量

    // vector<string> fileNames;
    
    string dir_path = "/media/vickylzy/文件共享盘/A3_研一/课程/计算机视觉/Assignment/Assignment1/shots/";
    Mat imageInput;
    Mat image;
    Mat imagesrc;
    Mat edge;
};
int main(int argc, char const **argv){
    Laneline_Detection lane;
    lane.read_images();
    lane.cannyedge(lane.fileNames[3]);
    lane.lane_detection();
}


    
    
