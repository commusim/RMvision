// OpenCVtest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
#include "stdio.h"
#include <iostream> 
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "function.h"
using namespace std;
using namespace cv;


const int kThreashold = 220;
const int kMaxVal = 255;
const Size kGaussianBlueSize = Size(5, 5);

// 相机内参
Mat cameraMatrix = (Mat_<double>(3, 3) <<
    2057.8,    0000.2,         0654.1,
         0,    2058.3,         0503.9,
         0,         0,         0001.0);
// 相机畸变系数
Mat dist_coeffs = (Mat_<double>(5, 1) << 
    -0.0003,    0.0012,
    -0.0877,    0.4026,   0.1095);


    
int main(){
    Mat frame;
    VideoCapture video;
    video.open("/home/commusim/RM/vision/video/手动曝光版.mp4");
    clock_t start,end;
    while(1){
        start = clock();
        // 读取视频序列中所有帧数的图像
        video >> frame;
        if (frame.empty()) {
            break;
        }
        // 完成装甲板识别得到对应像素点和实际点
        vector<Point2d> image_points = imp_reco(frame);
        //  vector<Point2d> image_points = Reco(frame);
        vector<Point3d> model_points;
        for(int i=0;i<4;i++){
            Point3d point = get3D(cameraMatrix,image_points[i].x,image_points[i].y,10);
            model_points.push_back(point);
        }
        // 实现PNP解算
        pair<Mat, Mat> vector;
        vector = posePNP(cameraMatrix,dist_coeffs,frame,image_points,model_points);
        Mat Rvec;
	    Mat_<float> Tvec;
        vector.first.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
	    vector.second.convertTo(Tvec, CV_32F); // 平移向量转换格式 

	    Mat_<float> rotMat(3, 3);
	    Rodrigues(Rvec, rotMat);
	    // 旋转向量转成旋转矩阵
	    cout << "rotMat" << endl << rotMat << endl << endl;

	    Mat P_oc;
	    P_oc = -rotMat.inv() * Tvec;
	    // 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm
	    cout << "P_oc" << endl << P_oc << endl;
        end = clock();
        cout<<"time = "<<double(end-start)/CLOCKS_PER_SEC*1000<<"ms"<<endl;  //输出时间（单位：ｓ）
    }

}

