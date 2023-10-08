
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

vector<Point2d> imp_reco(Mat frame){
    Mat channels[3],binary,Gaussian;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Rect boundRect;
    RotatedRect box;
    vector<Point2d> armor;
    
    Rect point_array[40];
    split(frame,channels);
    threshold(channels[0], binary, kThreashold, kMaxVal, 0);
    GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
    findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    int counter = 0,index[40] = {0};
    for (int i = 0; i < contours.size(); i++) {
        boundRect = boundingRect(Mat(contours[i]));
        try
        {
            if (double(boundRect.height / boundRect.width) >= 1.3 ) {
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
    int point_near[2]={-1};
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
    try{
        Point2f cache[4];
        if(point_near[0]!=-1&point_near[1]!=-1){
                for(int i=0;i<2;i++){
                RotatedRect ellipse = fitEllipse(contours[index[point_near[i]]]);
                double angle = ellipse.angle;
                Rect rect = boundingRect(contours[index[point_near[i]]]);
                RotatedRect rot(Point2f(rect.x+rect.width/2,rect.y+rect.height/2),
                    Size2f(rect.width,rect.height),angle);
                rot.points(cache);
                armor.push_back(cache[i*2]);
                armor.push_back(cache[i*2+1]);
                }
            if(abs(armor[0].x-armor[2].x)<250){
                for (int i = 0; i < 4; i++){
                    line(frame, armor[i], armor[(i + 1) % 4], Scalar(0, 255, 0), 2);
                }
                imshow("video", frame);
                waitKey(0);
                return armor;
                cout<< armor[0].x-armor[2].x<<endl;
            }   
        }
        
    }
    catch (const char* msg){
        cout << msg << endl;
        //continue;
    }
}
