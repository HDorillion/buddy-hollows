#include "myfreenectdevice.h"
#include <iostream>
#include <map>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include "objfollowing.h"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    bool debug = false;
    int thresh = 110;
    if(argc < 1) {
        cerr << "Usage: " << endl
             << "Kinected -options\n"
             << "\t -options\n"
             << "\t\t-d - debug mode on\n"
             << "\t\t-t (integer) - threshold value (1 - 255)\n";
        return 1;
    }
    for(int i = 0; i < argc; ++i){
        string option = argv[i];
        if(option == "-d"){
            debug = true;
        }
        else if(option == "-t"){
            if(i + 1 < argc){
                ++i;
                thresh = atoi(argv[i]);
            }
            else{
                cerr << "No value given for threshold\n";
            }
        }

    }

    // Define variables
    string hsvtrackbars = "HSV Trackbars";
    string rgbwindowname = "RGB", depthwindowname = "Depth", ROIwindowname = "ROI";
    MOTION_STATE current_motion = STATIONARY;
    TRACK_STATE current_track = FINDING;

    // Define image matrices
    Mat depthRef(Size(640,480),CV_16UC1);
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf(Size(640,480),CV_8UC1), depthOut(Size(640,480),CV_8UC1), depthDisp(Size(640,480),CV_8UC1);
    Mat depthrgb;

    Mat rgbRef(Size(640,480), CV_8UC3);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0)), rgbOut(Size(640,480),CV_8UC3,Scalar(0));
    Mat ROI;

    // Define freenect device
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    namedWindow(rgbwindowname, CV_WINDOW_AUTOSIZE);
    namedWindow(depthwindowname, CV_WINDOW_AUTOSIZE);

//    namedWindow(ROIwindowname,CV_WINDOW_AUTOSIZE);

    if(debug){
        // Create windows and trackbars
        namedWindow(hsvtrackbars, CV_WINDOW_AUTOSIZE);
        createTrackbar("Threshold", hsvtrackbars, &thresh, 255);
        createTrackbar("Hh", hsvtrackbars, &HSV.at("Hh"), 179);
        createTrackbar("Hl", hsvtrackbars, &HSV.at("Hl"), 179);
        createTrackbar("Sh", hsvtrackbars, &HSV.at("Sh"), 255);
        createTrackbar("Sl", hsvtrackbars, &HSV.at("Sl"), 255);
        createTrackbar("Vh", hsvtrackbars, &HSV.at("Vh"), 255);
        createTrackbar("Vl", hsvtrackbars, &HSV.at("Vl"), 255);
    }

    device.startDepth();
    device.startVideo();

    // Process data
    while(true){
        device.getDepth(depthMat);
        device.getVideo(rgbMat);

        depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2047.0);

        if(current_motion == STATIONARY && current_track == FINDING){
            depthf.copyTo(depthRef);
            rgbMat.copyTo(rgbRef);

            posterizeRGB(rgbRef, rgbOut);
            if(extractRGBROI(rgbOut, ROI,
                             Scalar(HSV.at("Hl"), HSV.at("Sl"), HSV.at("Vl")),
                             Scalar(HSV.at("Hh"), HSV.at("Sh"), HSV.at("Vh")))){
                // Do stuff
            }
//            if(extractDepthROI(depthRef, ROI, thresh)){
                // Image found
                // Set object up to be tracked
                    // On success, current_track = TRACKING
//            }
        }
        else if(current_motion == STATIONARY && current_track == TRACKING){
            // Follow stationary tracking algorithm
        }

//        differentiateObjects(rgbMat, rgbOut, thresh,
//                             Scalar(HSV.at("Hl"), HSV.at("Sl"), HSV.at("Vl")),
//                             Scalar(HSV.at("Hh"), HSV.at("Sh"), HSV.at("Vh")));


        // Combine images and show
//        rgbOut.copyTo(left);
//        depthOut.copyTo(right);
        imshow(rgbwindowname, rgbOut);
        if(!ROI.empty()){
            imshow(depthwindowname, ROI);
        }

        // Escape on escape
        char k = cvWaitKey(5);
        if( k == 27 ){
            break;
        }
    }

    // Cleanup
    destroyAllWindows();
    device.stopVideo();
    device.stopDepth();

    return 0;
}

