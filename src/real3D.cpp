#include<opencv4/opencv2/opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;
// 像素坐标 u
// 像素坐标 v
// 已知的深度信息 depth（三维点到相机的距离）
Point3d get3D(Mat cameraMatrix, double u, double v, double depth){ 
    // 将像素坐标转换为归一化坐标
    double x = (u - cameraMatrix.at<double>(0, 2)) / cameraMatrix.at<double>(0, 0);
    double y = (v - cameraMatrix.at<double>(1, 2)) / cameraMatrix.at<double>(1, 1);
    
    // 计算相机坐标系下的三维坐标
    double X = x * depth;
    double Y = y * depth;
    double Z = depth;

    Point3d model_point(X,Y,Z);

    return model_point;
}