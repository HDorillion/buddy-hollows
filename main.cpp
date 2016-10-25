#include "myfreenectdevice.h"
#include <stdio.h>
#include <map>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"

using namespace std;
using namespace cv;

// Global variables
map<string,int> HSV = {
    {"Hl", 10},
    {"Hh", 20},
    {"Sl", 125},
    {"Sh", 255},
    {"Vl", 100},
    {"Vh", 255}};

map<string,int> thresh = {
    {"Thresh", 110}
};

RNG rng(12345);

Mat findObject(Mat &orig);
Mat buildContours(Mat &orig);
Mat differentiateObjects(Mat &orig, Scalar HSVlb, Scalar HSVub);

int main(/*int argc, char *argv[]*/)
{
    // Define variables
    bool die = false;
    string filename("snapshot");
    string suffix(".png");
    string hsvtrackbars = "HSV Trackbars", rgbwindow = "RGB";
    int i_snap = 0;

    // Define image matrices
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));

    // Define freenect device
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    // Create windows and trackbars
    namedWindow(hsvtrackbars, CV_WINDOW_AUTOSIZE);
    namedWindow(rgbwindow,CV_WINDOW_AUTOSIZE);
    namedWindow("depth",CV_WINDOW_AUTOSIZE);
//    namedWindow("Contours", CV_WINDOW_AUTOSIZE);

    createTrackbar("Hh", hsvtrackbars, &HSV.at("Hh"), 179);
    createTrackbar("Hl", hsvtrackbars, &HSV.at("Hl"), 179);
    createTrackbar("Sh", hsvtrackbars, &HSV.at("Sh"), 255);
    createTrackbar("Sl", hsvtrackbars, &HSV.at("Sl"), 255);
    createTrackbar("Vh", hsvtrackbars, &HSV.at("Vh"), 255);
    createTrackbar("Vl", hsvtrackbars, &HSV.at("Vl"), 255);

    createTrackbar("Threshold:", rgbwindow, &thresh.at("Thresh"), 255);

    device.startVideo();
    device.startDepth();

    // Process data
    while (!die) {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
        imshow(rgbwindow,
               differentiateObjects(
                   rgbMat,
                   Scalar(HSV.at("Hl"), HSV.at("Sl"), HSV.at("Vl")),
                   Scalar(HSV.at("Hh"), HSV.at("Sh"), HSV.at("Vh"))));
//        imshow("Contours", buildContours(rgbMat));
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
            Scalar(HSV.at("Hl"), HSV.at("Sl"), HSV.at("Vl")),
            Scalar(HSV.at("Hh"), HSV.at("Sh"), HSV.at("Vh")),
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

// buildContours takes a matrix, finds contours, then adds
// bounding boxes to help track them
Mat buildContours(Mat &orig){
    // Define variables
    Mat orig_gray;
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Convert image to gray and blur it
    cvtColor(orig, orig_gray, CV_BGR2GRAY);
    blur(orig_gray, orig_gray, Size(3,3));

    /// Detect edges using Threshold
    threshold(orig_gray, threshold_output, thresh.at("Thresh"), 255, THRESH_BINARY);
    /// Find contours
    findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for(unsigned int i = 0; i < contours.size(); i++){
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, false);
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }

    /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for(unsigned int i = 0; i< contours.size(); i++){
        Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) );
        if(radius[i] > 7){
            drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
            circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
        }
    }
    return drawing;
}

// differentiateObjects takes an image, extracts a range of
// colors (HSV lower and upper bounds), and differentiates
// the objects of that color from each other.
Mat differentiateObjects(Mat &orig, Scalar HSVlb, Scalar HSVub){
    Mat imgHSV;
    cvtColor(orig, imgHSV, COLOR_BGR2HSV);

    // Morphological opening (remove fg objects)
    erode(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // Morphological closing (remove fg holes)
    dilate(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // Threshold image
    Mat colorThresh;
    inRange(imgHSV, HSVlb, HSVub, colorThresh);

    // Declare contour variables
    Mat HSV_gray;
    Mat threshout;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // Blur image
    blur(colorThresh, HSV_gray, Size(30,30));

    // Detect edges and find contours
    threshold(HSV_gray, threshout, thresh["Thresh"], 255, THRESH_BINARY);
    findContours(threshout, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


    // Declare polygon variables
    vector<vector<Point>> contours_poly(contours.size());
    vector<Rect> boundRects(contours.size());
    vector<Point2f> center(contours.size());
    vector<float> radius(contours.size());

    // Declare moment variables
    vector<Moments> objMomentsVec(contours.size());
    vector<Point> objCentroids(contours.size());

    // Approximate contours to polygons and build bounding rectangles
    for(unsigned int i = 0; i < contours.size(); ++i){
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, false);
        boundRects[i] = boundingRect(Mat(contours_poly[i]));
        minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
    }

    // Draw bounding rectangles
    Mat overlay = orig;
    for(unsigned int i = 0; i < contours.size(); ++i){
        Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
        if(radius[i] > 15){
            drawContours(overlay, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
//            rectangle(overlay, boundRects[i].tl(), boundRects[i].br(), color, 2, 8, 0);
            objMomentsVec[i] = moments(HSV_gray(boundRects[i]).clone());
            if(objMomentsVec[i].m00 > 10000){
                objCentroids[i] = Point(objMomentsVec[i].m10 / objMomentsVec[i].m00,
                                        objMomentsVec[i].m01 / objMomentsVec[i].m00) + boundRects[i].tl();
                circle(overlay, objCentroids[i], 2, Scalar(0,255,0),-1);
            }
        }
    }
    return overlay;
}
