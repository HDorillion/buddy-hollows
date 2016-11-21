#include "myfreenectdevice.h"
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"

#if CV_MAJOR_VERSION == 3
    #include "opencv2/videoio.hpp"
#else
    #include "opencv2/video.hpp"
#endif

#include "objfollowing.h"

using namespace std;
using namespace cv;

void captureWebcam(int &num_objects);
void captureKinect(int &num_objects);

// main ...
int main(int argc, char *argv[])
{
    // Logic flow variables
    bool debug = false, webcam = false;
    int thresh = 110, num_objects = 3;
    string hsvtrackbars = "Debug";

    // Display usage on failure
    if(argc < 1) {
        cerr << "Usage: " << endl
             << "Kinected -options\n"
             << "\t -options\n"
             << "\t\t-d - debug mode on\n"
             << "\t\t-t (integer) - threshold value (1 - 255)\n"
             << "\t\t-c - webcam mode on\n";
        return 1;
    }

    // Parse command line
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
        else if(option == "-c"){
            webcam = true;
        }
    }

    // Debug sliders
    if(debug){
        // Create windows and trackbars
        namedWindow(hsvtrackbars, CV_WINDOW_AUTOSIZE);
//        createTrackbar("Threshold", hsvtrackbars, &thresh, 255);
        createTrackbar("Hh", hsvtrackbars, &HSV.at("Hh"), 179);
        createTrackbar("Hl", hsvtrackbars, &HSV.at("Hl"), 179);
        createTrackbar("Sh", hsvtrackbars, &HSV.at("Sh"), 255);
        createTrackbar("Sl", hsvtrackbars, &HSV.at("Sl"), 255);
        createTrackbar("Vh", hsvtrackbars, &HSV.at("Vh"), 255);
        createTrackbar("Vl", hsvtrackbars, &HSV.at("Vl"), 255);
    }

    // Enter main logic loop
    if(webcam) captureWebcam(num_objects);
    else captureKinect(num_objects);
    return 0;
}

// captureWebcam captures only the webcam and tracks "an" object
// of a certain color
void captureWebcam(int &num_objects){
    // Define strings
    string rgbwindowname = "RGB";

    // RGB image matrices
    Mat rgbRef(Size(640,480), CV_8UC3);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0)), rgbOut(Size(640,480),CV_8UC3,Scalar(0));
    Mat ROI;

    // Object tracking variables
    Point center = Point(640 / 2, 480 / 2);

    //Define webcam
    VideoCapture webcap = VideoCapture(0);

    int n = 0;
    while(!webcap.isOpened()){
        webcap = VideoCapture(n);
        if(n >= 5) exit(EXIT_FAILURE);
    }
    namedWindow(rgbwindowname, CV_WINDOW_AUTOSIZE);

    // Process data
    while(true){
        // Read image
        if(!webcap.read(rgbMat)) continue;

        // Process images
        rgbMat.copyTo(rgbRef);
        if(discernObject(rgbMat,
                        Scalar(HSV.at("Hl"), HSV.at("Sl"), HSV.at("Vl")),
                        Scalar(HSV.at("Hh"), HSV.at("Sh"), HSV.at("Vh")),
                        num_objects, center, ROI)){
                // Do stuff
        }

        // Show images
        if(!ROI.empty()){
            imshow(rgbwindowname, ROI);
        }

        // Escape on escape
        if(cvWaitKey(10) == 27){
            break;
        }
    }
    // Cleanup
    destroyAllWindows();
}

// captureKinect captures the Kinect's camera and depth data
// and tracks "an" object of a certain color
void captureKinect(int &num_objects){
    // Define strings
    string rgbwindowname = "RGB", depthwindowname = "Depth";

    // Depth image matrices
    Mat depthRef(Size(640,480),CV_16UC1);
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf(Size(640,480),CV_8UC1), depthOut(Size(640,480),CV_8UC1);

    // RGB image matrices
    Mat rgbRef(Size(640,480), CV_8UC3);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0)), rgbOut(Size(640,480),CV_8UC3,Scalar(0));
    Mat ROI;

    // Object tracking variables
    Point center = Point(640 / 2, 480 / 2);

    // Define freenect device
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    // Start data gathering
    device.startDepth();
    device.startVideo();

    // Create window names
    namedWindow(depthwindowname, CV_WINDOW_AUTOSIZE);
    namedWindow(rgbwindowname, CV_WINDOW_AUTOSIZE);

    // Process data
    while(true){
        // Get Kinect data
        device.getDepth(depthMat);
        device.getVideo(rgbMat);
        depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2047.0);
        depthf.copyTo(depthRef);

        // Process images
        rgbMat.copyTo(rgbRef);
        if(discernObject(rgbRef,
                        Scalar(HSV.at("Hl"), HSV.at("Sl"), HSV.at("Vl")),
                        Scalar(HSV.at("Hh"), HSV.at("Sh"), HSV.at("Vh")),
                        num_objects, center, ROI)){
                // Do stuff
        }

        // Show images
        if(!ROI.empty()){
            imshow(depthwindowname, ROI);
        }
        if(!rgbMat.empty()){
            imshow(rgbwindowname, rgbMat);
        }

        // Escape on escape
        if(cvWaitKey(10) == 27){
            break;
        }
    }

    // Cleanup
    device.stopVideo();
    device.stopDepth();
    destroyAllWindows();
}
