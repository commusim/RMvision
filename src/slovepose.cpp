// OpenCVtest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
#include "stdio.h"
#include <iostream> 
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


std::pair<Mat, Mat> posePNP(Mat camera_matrix,Mat dist_coeffs,Mat image,
	vector<Point2d> image_points,vector<Point3d> model_points);
Point3d get3D(Mat cameraMatrix, double u, double v, double depth);
vector<Point2d> Recog(Mat frame);

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
    while(1){
        // 读取视频序列中所有帧数的图像
        video >> frame;
        if (frame.empty()) {
            break;
        }
        // 完成装甲板识别得到对应像素点和实际点
        vector<Point2d> image_points = Recog(frame);
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
    }
}



std::pair<Mat, Mat> posePNP(Mat camera_matrix,Mat dist_coeffs,Mat image,
	vector<Point2d> image_points,vector<Point3d> model_points){
	
	// 旋转向量
	Mat rotation_vector;
	// 平移向量
	Mat translation_vector;
	
	// pnp求解
	solvePnP(model_points, image_points, camera_matrix, dist_coeffs, 
		rotation_vector, translation_vector, 0, SOLVEPNP_ITERATIVE);
	// 默认ITERATIVE方法，可尝试修改为EPNP（CV_EPNP）,P3P（CV_P3P）

	cout << "Rotation Vector " << endl << rotation_vector << endl << endl;
	cout << "Translation Vector" << endl << translation_vector << endl << endl;

	imshow("Output", image);
	waitKey(0);
	
	return std::make_pair(rotation_vector, translation_vector);
}

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

vector<Point2d> Recog(Mat frame){
    Mat channels[3],binary,Gaussian;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Rect boundRect;
    RotatedRect box;
    vector<Point2f> boxPts(4);  
        Rect point_array[20];
        // 
        split(frame,channels);
        threshold(channels[0], binary, kThreashold, kMaxVal, 0);
        GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
        findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
        int index = 0;
    for (int i = 0; i < contours.size(); i++) {
        //box = minAreaRect(Mat(contours[i]));
        //box.points(boxPts.data());
        boundRect = boundingRect(Mat(contours[i]));
        //rectangle(frame, boundRect.tl(), boundRect.br(), (0, 255, 0), 2,8 ,0);
        if(double(boundRect.height / boundRect.width) >= 1.3 && boundRect.height > 36 && boundRect.height>20) {
                point_array[index] = boundRect;
                index++;
            }
        }
        int point_near[2];
        // 找到最小面积
    for (int i = 0; i < index-1; i++){
        int min = 10000;
        for (int j = i + 1; j < index; j++) {
            int value = abs(point_array[i].area() - point_array[j].area());
            if (value < min){
                min = value;
                point_near[0] = i;
                point_near[1] = j;
            }
        }
    }

    Rect rectangle_1 = point_array[point_near[0]];
    Rect rectangle_2 = point_array[point_near[1]];
    if (rectangle_2.x == 0 || rectangle_1.x == 0) {
        throw "not enough points";
    }
        // 成功找到像素点位置
    Point point1 = Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y);
    Point point2 = Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y + rectangle_1.height);
    Point point3 = Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y);
    Point point4 = Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y + rectangle_2.height);
    vector<Point2d> p = { point1,point2,point4,point3 };
            
    cout << p[0]<<p[1]<<p[2]<<p[3] << endl;
    for (int i = 0; i < 4; i++) {
        line(frame, p[i%4], p[(i+1)%4], Scalar(0, 255, 0), 2);
    }           
    return p;
}