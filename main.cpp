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
    // Define variables
    bool die = false;
    string filename("snapshot");
    string suffix(".png");
    int i_snap = 0;

    // Define image matrices
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));

    // Prepare preprocessing directive
    Mat imgLines = Mat::zeros(Size(640,480), CV_8UC3);

    // Define freenect device
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    // lastposit[] holds data of object's last position
    int lastposit[2] = {0, 0};

    // Create windows and begin capturing data
    namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    namedWindow("depth",CV_WINDOW_AUTOSIZE);
    device.startVideo();
    device.startDepth();

    // Process data
    while (!die) {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
        imshow("rgb", findObject(imgLines, rgbMat, (&lastposit)[0]));
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
Mat findObject(Mat &tracklines, Mat &orig, int (&lastposit)[2]){
    Mat imgHSV;
    cvtColor(orig, imgHSV, COLOR_BGR2HSV);

    // Threshhold the image
    Mat imgThresh;
    inRange(imgHSV, Scalar(160, 100, 100), Scalar(179, 255, 255), imgThresh);

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

        // Draw tracking line
        if(lastposit[0] >= 0 && lastposit[1] >= 0 && posX >= 0 && posY >= 0){
            line(tracklines, Point(posX, posY), Point(lastposit[0], lastposit[1]), Scalar(0,255,255));
        }

        // Index position vector
        lastposit[0] = posX;
        lastposit[1] = posY;
    }

    // Return tracklines
    return orig + tracklines;
}
