#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
using namespace cv;

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

	Mat Rvec;
	Mat_<float> Tvec;
	rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
	translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 

	Mat_<float> rotMat(3, 3);
	Rodrigues(Rvec, rotMat);
	// 旋转向量转成旋转矩阵
	// cout << "rotMat" << endl << rotMat << endl << endl;

	Mat P_oc;
	P_oc = -rotMat.inv() * Tvec;
	// 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm
	cout << "P_oc" << endl << P_oc << endl;
	/*
	imshow("Output", image);
	waitKey(0);
	*/
	
	return std::make_pair(rotation_vector, translation_vector);
}
