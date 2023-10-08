// OpenCVtest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "stdio.h"
#include <iostream> 
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
const int kThreashold = 220;
const int kMaxVal = 255;
const Size kGaussianBlueSize = Size(5, 5);
int main()
{
    VideoCapture video;
    video.open("/home/commusim/RM/vision/video/手动曝光版.mp4");
    Mat frame,channels[3],binary,Gaussian;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Rect boundRect;
    RotatedRect box;
    vector<Point2f> boxPts(4);
    
    for (;;) {
        Rect point_array[20];
        video >> frame;
        if (frame.empty()) {
            break;
        }
        split(frame,channels);
        threshold(channels[0], binary, kThreashold, kMaxVal, 0);
        GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
        findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
        int counter = 0,index[20];
        for (int i = 0; i < contours.size(); i++) {
            //box = minAreaRect(Mat(contours[i]));
           //box.points(boxPts.data());
            boundRect = boundingRect(Mat(contours[i]));
            //rectangle(frame, boundRect.tl(), boundRect.br(), (0, 255, 0), 2,8 ,0);
            try
            {
                if (double(boundRect.height / boundRect.width) >= 1.3 && boundRect.height > 36 && boundRect.height>20) {
                    point_array[counter] = boundRect;
                    index[counter]=i;
                    counter++;
                }
            }
            catch (const char* msg)
            {
                cout << printf(msg) << endl;
                //continue;
            }
        }        
        int point_near[2];
        int min = 10000;
        for (int i = 0; i < counter-1; i++)
        {
            for (int j = i + 1; j < counter; j++) {
                int value = abs(point_array[i].area() - point_array[j].area());
                if (value < min)
                {
                    min = value;
                    point_near[0] = i;
                    point_near[1] = j;
                }
            }
        }   
        try
        {
            RotatedRect ellipse_1 = fitEllipse(contours[index[point_near[0]]]);  // 使用椭圆拟合轮廓
            RotatedRect ellipse_2 = fitEllipse(contours[index[point_near[1]]]);  // 使用椭圆拟合轮廓
            // 根据椭圆获取灯条的倾斜角度
            double angle_1 = ellipse_1.angle;
            double angle_2 = ellipse_2.angle;
            // 使用矩形拟合轮廓，获取大小和中心坐标
            Rect rect_1 = boundingRect(contours[index[point_near[0]]]);
            Rect rect_2 = boundingRect(contours[index[point_near[1]]]);
            RotatedRect rot_1(Point2f(rect_1.x+rect_1.width/2,rect_1.y+rect_1.height/2),
                Size2f(rect_1.width,rect_1.height),angle_1);
            RotatedRect rot_2(Point2f(rect_2.x+rect_2.width/2,rect_2.y+rect_2.height/2),
                Size2f(rect_2.width,rect_2.height),angle_2);
            Point2f vertices[4];
    	rot_1.points(vertices);
	    for (int i = 0; i < 4; i++)
		    line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2);
        rot_2.points(vertices);
        for (int i = 0; i < 4; i++)
		    line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2);
        }
        catch (const char* msg)
        {
            cout << msg << endl;
            //continue;
        }
        imshow("video", frame);
        if (waitKey(10) >= 0) {
            break;
        }
    }
    video.release();
    cv::destroyAllWindows();
    return 0;
}