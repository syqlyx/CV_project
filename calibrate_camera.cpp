#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"
#include <sys/stat.h>

using namespace std;
using namespace cv;
                                       
int width = 8;
int height = 6;

class Calibrate_Camera{
public:
    void read_images(){
        Directory dir;
        fileNames = dir.GetListFiles(dir_path,"*.jpg", false);
        // std::string jpg = "/media/vickylzy/文件共享盘/A3_研一/课程/计算机视觉/Assignment1/Mode1/*.jpg";
        // std::vector<cv::String> fileNames;//名字列表
        // cv::glob(jpg, fileNames);
    }


    void get_corner(){
        criteria = TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 );//精度到0.1或者次数到30，亚像素角点检测算法便退出
        cvtColor(imageInput,image,CV_RGB2GRAY);
        found = findChessboardCorners(image, patternSize, image_points_buf);
        if(found){
            cornerSubPix(image, image_points_buf, winSize, zeroZOne, criteria);//image_points_buf会被重写
            drawChessboardCorners(image, patternSize, image_points_buf, found);//image会被重写
            image_points.push_back(image_points_buf);
        }
    }


    void calibration(){
        for(int count=0; count < fileNames.size(); count ++){
            string filename = dir_path + fileNames(i);
            image_count ++;
            Mat imageInput = cv::imread(filename);
            cv::cvtColor(imageInput, image, CV_BGR2GRAY);
            if (image_count == 1)  //读入第一张图片时获取图像宽高信息
            {
                image_size.width = imageInput.cols;//整张图片的像素尺寸???????????????????????????????
                image_size.height =imageInput.rows;	
            }
            //生成图片的实际三维坐标
            vector<Point3f> Points; 
            for(i=0; i<width; i++){
                for(j=0; j<height; j++){
                    Point3f point;
                    point.x = (float)i * square_size.width;
                    point.y = (float)j * square_size.height;
                    point.z = 0;
                    Points.push_back(point);
                }
            }
            object_points.push_back(Points);
        }
        calibrateCamera(object_points, image_points, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
    }

/**
 * 对标定结果进行评价的方法是通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到空间三维点在图像上新的投影点的坐标，
 * 计算投影坐标和亚像素角点坐标之间的偏差，偏差越小，标定结果越好。
**/
    void project(){
        projectPoints(object_points, rvecsMat, tvecsMat, cameraMatrix, distCoeffs, image_points_projection);
        //error
        undistort(image, image, cameraMatrix, distCoeffs, cameraMatrix);
    }


private:
    int width = 8;
    int height = 6;
    bool found = false;
    Size winSize = cv::Size(5,5);
    Size = cv::Size(-1,-1);
    TermCriteria criteria;
    Size patternSize = cv::Size(width, height);
    vector<Point2f> image_points_buf;//一张图片经过角点提取，得到的所有角点坐标
    vector<vector<Point2f>> image_points;//所有图像的角点坐标
    vector<vector<Point2f>> image_points_projection;//所有图像的角点坐标
    int image_count = 0;
    Size square_size = Size(10,10);  /* 实际测量得到的标定板上每个棋盘格的大小 */
    vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
    /*内外参数*/
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */
    vector<int> point_counts;  // 每幅图像中角点的数量
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    vector<Mat> tvecsMat;  /* 【每幅】图像的平移向量 */
    vector<Mat> rvecsMat; /* 【每幅】图像的旋转向量 */
    vector<vector<Point3f>> object_points;//叠加
    Size image_size;
    vector<string> fileNames;
    string dir_path = "/media/vickylzy/文件共享盘/A3_研一/课程/计算机视觉/Assignment1/Mode1/";
};
int main(int argc, char const **argv){
    Calibrate_Camera cali;
    cali.read_images();
    cali.get_corner();
    cali.calibration();
    cali.project();
}

