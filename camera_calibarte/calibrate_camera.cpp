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

class Calibrate_Camera{
public:
    void read_images(){
        // cv::Directory dir;//opencv 类
        // fileNames = dir.GetListFiles(dir_path,"*.jpg", false);
        cv:String jpg = "/media/vickylzy/文件共享盘/A3_grade_one/class/computer_vision/Assignment1/Mode2";
        // std::vector<cv::String> fileNames;//名字列表
        cv::glob(jpg, fileNames);
    }


    bool get_corner(const cv::String& filename){
        imageInput = cv::imread(filename);
        if(!imageInput.data){
            cout << "error in loading picture" << endl;
            return false;
        }
        cv::cvtColor(imageInput, image, CV_RGB2GRAY);
        found = findChessboardCorners(image, patternSize, image_points_buf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);//8,6
        if(found){
            cornerSubPix(image, image_points_buf, winSize, zeroZone, criteria);//image_points_buf会被重写
            drawChessboardCorners(image, patternSize, image_points_buf, found);//image会被重写
            // cv::imshow("corner",image);//显示图片
			// waitKey(50);//暂停0.5S
            image_points.push_back(image_points_buf);
            return true;
        }
        return false;
    }


    void calibration(){
        for(size_t count = 0; count < 10; count ++){
            cv::String filename = fileNames[count];//vector [i]
            image_count ++;
            if(!get_corner(filename)){
                continue;
            }
            cout << "image_count" << image_count << endl;
            if (image_count == 1)  //读入第一张图片时获取图像宽高信息
            {
                image_size = image.size();
            }
            //生成图片的实际三维坐标
            vector<Point3f> Points; 
            for(int i=0; i < height; i++){
                for(int j=0; j < width; j++){
                    Point3f point;
                    point.x = (float)j * square_size.width;
                    point.y = (float)i * square_size.height;
                    point.z = 0.0;
                    Points.push_back(point);
                }
            }
            object_points.push_back(Points);
        }
        calibrateCamera(object_points, image_points, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
        cout << cameraMatrix << endl;
        cout << distCoeffs << endl;
        // getoptimal  裁剪
        newMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image_size, 1, image_size, 0);
        // cout << newMatrix << endl;
    }

/**
 * 对标定结果进行评价的方法是通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到空间三维点在图像上新的投影点的坐标，
 * 计算投影坐标和亚像素角点坐标之间的偏差，偏差越小，标定结果越好。
**/
    void project(){
        for(size_t count = 0; count < 10; count ++){
            vector<Point3f> objp = object_points[count];
            vector<Point2f> imgp = image_points[count];
            Mat r = rvecsMat[count];
            Mat t = tvecsMat[count];
            projectPoints(objp, r, t, cameraMatrix, distCoeffs, imgp_projection);
            float err = cv::norm(Mat(imgp), Mat(imgp_projection), cv::NORM_L2);
            float error = sqrt(err*err/48);
            cout<<error<<endl;
        }
        //error
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                newMatrix, image_size, CV_16SC2, map1, map2);
        for(size_t count = 0; count < 10; count ++){
            cv::String filename = fileNames[count];//vector [i]
            cv::cvtColor(cv::imread(filename), image, CV_RGB2GRAY);
            undistort(image, new_image, cameraMatrix, distCoeffs, cameraMatrix);
            // remap(image, new_image, map1, map2, INTER_LINEAR);
            cv::imshow("image",new_image);
            cv::waitKey();
        }
    }
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
    std::vector<cv::String> fileNames;
    string dir_path = "/media/vickylzy/文件共享盘/A3_研一/课程/计算机视觉/Assignment1/shots/";
    Mat imageInput;
    Mat image;
    Mat new_image;
};
int main(int argc, char const **argv){
    Calibrate_Camera cali;
    cali.read_images();
    cali.calibration();
    cali.project();
}

