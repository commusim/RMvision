#include<iostream>
#include<opencv4/opencv2/opencv.hpp>
using namespace std;
using namespace cv;

std::pair<Mat, Mat> posePNP(Mat camera_matrix,Mat dist_coeffs,Mat image,
	vector<Point2d> image_points,vector<Point3d> model_points);
Point3d get3D(Mat cameraMatrix, double u, double v, double depth);
vector<Point2d> Recog(Mat frame);