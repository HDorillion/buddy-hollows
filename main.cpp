#include "myfreenectdevice.h"
#include <stdio.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"

using namespace std;
using namespace cv;

Mat findObject(Mat &background, Mat &orig, int (&lastposit)[2]);

int main(/*int argc, char *argv[]*/)
{
    bool die = false;
    string filename("snapshot");
    string suffix(".png");
    int i_snap = 0;

    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));

    // Prepare preprocessing directive
    Mat imgLines = Mat::zeros(Size(640,480), CV_8UC3);

    // The next two lines must be changed as Freenect::Freenect
    // isn't a template but the method createDevice:
    // Freenect::Freenect<MyFreenectDevice> freenect;
    // MyFreenectDevice& device = freenect.createDevice(0);
    // by these two lines:

    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    int lastposit[2] = {0, 0};

    namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    namedWindow("depth",CV_WINDOW_AUTOSIZE);
    device.startVideo();
    device.startDepth();
    while (!die) {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
        imshow("rgb", findObject(imgLines, rgbMat, (&lastposit)[0]));
        depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        imshow("depth",depthf);
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
    destroyAllWindows();
    device.stopVideo();
    device.stopDepth();
    return 0;
}

// Find single object of particular color
Mat findObject(Mat &tracklines, Mat &orig, int (&lastposit)[2]){
    Mat imgHSV;
    cvtColor(orig, imgHSV, COLOR_BGR2HSV);

    // Threshhold the image
    Mat imgThresh;
    inRange(imgHSV, Scalar(0, 100, 100), Scalar(10, 255, 255), imgThresh);

    // Morphological opening (remove fg objects)
    erode(imgThresh, imgThresh, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgThresh, imgThresh, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // Morphological closing (remove fg holes)
    dilate(imgThresh, imgThresh, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(imgThresh, imgThresh, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // Calculate moment
    Moments objMoments = moments(imgThresh);
    double dM01 = objMoments.m01;
    double dM10 = objMoments.m10;
    double dArea = objMoments.m00;

    // Determine if object exists
    if(dArea > 10000){
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;

        printf("X: %d, Y: %d\n", posX, posY);

        if(lastposit[0] >= 0 && lastposit[1] >= 0 && posX >= 0 && posY >= 0){
            line(tracklines, Point(posX, posY), Point(lastposit[0], lastposit[1]), Scalar(0,255,255));
        }

        lastposit[0] = posX;
        lastposit[1] = posY;
    }

    // Return tracklines
    return orig + tracklines;
}
