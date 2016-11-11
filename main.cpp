#include "myfreenectdevice.h"
#include <iostream>
#include <map>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"

using namespace std;
using namespace cv;

// Global variables
map<string,int> HSV {
    {"Hl", 10},
    {"Hh", 20},
    {"Sl", 125},
    {"Sh", 255},
    {"Vl", 100},
    {"Vh", 255}
};

RNG rng(12345);
enum MOTION_STATE {STATIONARY = 0, MOVING = 1};
enum TRACK_STATE {FINDING = 0, TRACKING = 1};

bool extractDepthROI(Mat &depthsrc, Mat &ROI, Rect ROIrect, int &thresh);
bool pair_is_less(pair<int, double> i, pair<int, double> j);
vector<Point> contoursConvexHull(vector<vector<Point>> contours);
void buildOutlinesFromDepth(Mat &graysrc, Mat &dst, int &thresh);
void buildContours(Mat &src, Mat &dst, int &thresh);
void differentiateObjects(Mat &src, Mat &dst, int &thresh, Scalar HSVlb, Scalar HSVub);

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
                cerr << "Threshold value must imediately follow use of -t\n";
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
    Rect ROIrect;

//    Mat displayMat, left, right;
//    displayMat = Mat(depthOut.rows, depthOut.cols + rgbOut.cols, CV_8UC3);
//    left = Mat(displayMat, Rect(0, 0, rgbOut.cols, depthOut.rows));
//    right = Mat(displayMat, Rect(rgbOut.cols, 0, depthOut.cols, depthOut.rows));

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
            if(extractDepthROI(depthRef, ROI, ROIrect, thresh)){
                // Image found
                // Set object up to be tracked
                    // On success, current_track = TRACKING
                rectangle(rgbOut, ROIrect, Scalar(0,255,0), 8);
            }
        }
        else if(current_motion == STATIONARY && current_track == TRACKING){
            // Follow stationary tracking algorithm
        }

        differentiateObjects(rgbMat, rgbOut, thresh,
                             Scalar(HSV.at("Hl"), HSV.at("Sl"), HSV.at("Vl")),
                             Scalar(HSV.at("Hh"), HSV.at("Sh"), HSV.at("Vh")));


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

bool extractDepthROI(Mat &depthsrc, Mat &ROI, Rect ROIrect, int &thresh){
    Mat threshMat;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    // Threshold image to find the most likely person
    GaussianBlur(depthsrc, depthsrc, Size(15,15), 7);
    threshold(depthsrc, threshMat, thresh, 255, THRESH_BINARY);
    adaptiveThreshold(threshMat, threshMat, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 15, -5);
//    Canny(depthsrc, threshMat, 20, 60, 3);
    findContours(threshMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point>> hull(contours.size());
    vector<vector<Point>> contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> center(contours.size());
    vector<float> radius(contours.size());

    // Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros(threshMat.size(), CV_8UC1);
    Scalar color = rng.uniform(0,255);
    vector<pair<int, double>> bigvex;
    for(unsigned int i = 0; i< contours.size(); ++i){
        convexHull(Mat(contours[i]), hull[i], false);
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, false);
        boundRect[i] = boundingRect( Mat(contours_poly[i]));
//        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i]);
//            drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
//            rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
//            circle(dst, center[i], (int)radius[i], color, 2, 8, 0);
//        }
        double localArea = contourArea(contours_poly[i]);
        bigvex.push_back(make_pair(i, localArea));
    }

    sort(begin(bigvex), end(bigvex), pair_is_less);

//    for(unsigned int i = 0; i < bigvex.size() / 3; ++i){
//        rectangle(drawing, boundRect[bigvex.at(i).first].tl(), boundRect[bigvex.at(i).first].br(), color, 1, 8);
//        drawContours(drawing, hull, bigvex.at(i).first, color, 1, 8, vector<Vec4i>(), 0, Point());
//        ROI = depthsrc(Rect(boundRect[bigvex.at(i).first].tl(), boundRect[bigvex.at(i).first].br()));
//        break;
//    }

    for(unsigned int i = 0; i < bigvex.size() / 3; ++i){
        if(bigvex.size() >= 2){
            Point upperleft = (boundRect[bigvex.at(i).first].tl() + boundRect[bigvex.at(i + 1).first].tl()) / 2;
            Point bottomright = (boundRect[bigvex.at(i).first].br() + boundRect[bigvex.at(i + 1).first].br()) / 2;
            ROIrect = Rect(upperleft, bottomright);
            ROI = depthsrc(ROIrect);
            break;
        }
    }

//    drawing.copyTo(ROI);
//    ROI = threshMat + ROI;

    if(!ROI.empty()){
        return true;
    }
    return false;
}

bool pair_is_less(pair<int, double> i, pair<int, double> j){
    return i.second > j.second;
}

vector<Point> contoursConvexHull(vector<vector<Point>> contours){
    vector<Point> result;
    vector<Point> pts;
    for ( size_t i = 0; i < contours.size(); i++)
        for ( size_t j = 0; j < contours[i].size(); j++)
            pts.push_back(contours[i][j]);
    convexHull( pts, result );
    return result;
}

void buildOutlinesFromDepth(Mat &graysrc, Mat &dst, int &thresh){

}

// buildContours takes a matrix, finds contours, then adds
// bounding boxes to help track them
void buildContours(Mat &graysrc, Mat &dst, int &thresh){
    // Define variables
    Mat threshold_output;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // Blur image
    blur(graysrc, graysrc, Size(30,30));

    // Threshold to detect contours
    threshold(graysrc, threshold_output, thresh, 255, THRESH_BINARY);
    findContours(graysrc, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size());
    vector<Rect> boundRect( contours.size());
    vector<Point2f>center( contours.size());
    vector<float>radius( contours.size());

    for(unsigned int i = 0; i < contours.size(); ++i){
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, false);
        boundRect[i] = boundingRect( Mat(contours_poly[i]));
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i]);
    }

    // Draw polygonal contour + bonding rects + circles
    for(unsigned int i = 0; i< contours.size(); ++i){
        Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
        if(radius[i] > 7){
            drawContours(dst, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
            rectangle(dst, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
            circle(dst, center[i], (int)radius[i], color, 2, 8, 0);
        }
    }
}

// differentiateObjects takes an image, extracts a range of
// colors (HSV lower and upper bounds), and differentiates
// the objects of that color from each other.
void differentiateObjects(Mat &src, Mat &dst, int &thresh, Scalar HSVlb, Scalar HSVub){
    Mat imgHSV;
    dst = 0;
    cvtColor(src, imgHSV, COLOR_BGR2HSV);

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
    threshold(HSV_gray, threshout, thresh, 255, THRESH_BINARY);
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
    for(unsigned int i = 0; i < contours.size(); ++i){
        Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
        if(radius[i] > 15){
            drawContours(dst, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
//            rectangle(overlay, boundRects[i].tl(), boundRects[i].br(), color, 2, 8, 0);
            objMomentsVec[i] = moments(HSV_gray(boundRects[i]).clone());
            if(objMomentsVec[i].m00 > 10000){
                objCentroids[i] = Point(objMomentsVec[i].m10 / objMomentsVec[i].m00,
                                        objMomentsVec[i].m01 / objMomentsVec[i].m00) + boundRects[i].tl();
                circle(dst, objCentroids[i], 2, Scalar(0,255,0),-1);
            }
        }
    }
    dst += src;
}
