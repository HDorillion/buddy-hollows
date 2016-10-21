#include "myfreenectdevice.h"
#include <stdio.h>
#include <map>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"

using namespace std;
using namespace cv;

// Global variables
map<string,int> BGR = {
    {"bl", 175},
    {"bh", 255},
    {"gl", 70},
    {"gh", 200},
    {"rl", 145},
    {"rh", 255}};

Mat findObject(Mat &orig);

int main(/*int argc, char *argv[]*/)
{
    // Define variables
    bool die = false;
    string filename("snapshot");
    string suffix(".png");
    string rgbwindow = "RGB";
    int i_snap = 0;

    // Define image matrices
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));

    // Define freenect device
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    // Create windows and trackbars
    namedWindow(rgbwindow,CV_WINDOW_AUTOSIZE);
    namedWindow("depth",CV_WINDOW_AUTOSIZE);

    createTrackbar("rh", rgbwindow, &BGR.at("rh"), 255);
    createTrackbar("rl", rgbwindow, &BGR.at("rl"), 255);
    createTrackbar("gh", rgbwindow, &BGR.at("gh"), 255);
    createTrackbar("gl", rgbwindow, &BGR.at("gl"), 255);
    createTrackbar("bh", rgbwindow, &BGR.at("bh"), 255);
    createTrackbar("bl", rgbwindow, &BGR.at("bl"), 255);

    device.startVideo();
    device.startDepth();

    // Process data
    while (!die) {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
        imshow(rgbwindow, findObject(rgbMat));
        depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        imshow("depth",depthf);

        // Escape on escape
        char k = cvWaitKey(5);
        if( k == 27 ){
            break;
        }
        if( k == 8 ) {
            std::ostringstream file;
            file << filename << i_snap << suffix;
            cv::imwrite(file.str(),rgbMat);
            i_snap++;
        }
    }

    // Cleanup
    destroyAllWindows();
    device.stopVideo();
    device.stopDepth();

    return 0;
}

// Find single object of particular color
Mat findObject(Mat &orig){
    Mat imgHSV;
    cvtColor(orig, imgHSV, COLOR_BGR2HSV);

    // Morphological opening (remove fg objects)
    erode(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // Morphological closing (remove fg holes)
    dilate(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // Threshhold the image
    Mat imgThresh;
    inRange(imgHSV,
            Scalar(BGR.at("rl"), BGR.at("gl"), BGR.at("bl")),
            Scalar(BGR.at("rh"), BGR.at("gh"), BGR.at("bh")),
            imgThresh);

    // Calculate moment
    Moments objMoments = moments(imgThresh);
    double dM01 = objMoments.m01;
    double dM10 = objMoments.m10;
    double dArea = objMoments.m00;

    // Determine if object exists
    if(dArea > 10000){
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;
        circle(orig, Point(posX,posY), 8, Scalar(0,255,0), -1, 8);
    }

    // Return tracklines
    return orig;
}
