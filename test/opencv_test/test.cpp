#include<opencv2/opencv.hpp>
#include<iostream>
 
using namespace cv;
using namespace std;
 
int main()
{
	string path = "/home/commusim/RM/vision/images/Snapshot-202212012101121287781.JPG";//图片的路径名
	Mat img = imread(path);
	imshow("Image", img);//创建一个窗口来显示图像img
    waitKey(0);
	return 0;
}