#include <cmath>

#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"

#include "objfollowing.h"

// Map of HSV values
map<string,int> HSV {
    {"Hl", 55},
    {"Hh", 80},
    {"Sl", 110},
    {"Sh", 255},
    {"Vl", 100},
    {"Vh", 255}
};

// Color variable range
RNG rng(12345);

// discernObject
bool discernObject(Mat &src, const Scalar lb, const Scalar ub, const int &num_objects, Point &center, Mat &ROI){
    Mat colorThresh, threshMat;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    Mat HSVMat;
    cvtColor(src, HSVMat, COLOR_BGR2HSV);

    // Find contours within color range
    inRange(HSVMat, lb, ub, colorThresh);
    GaussianBlur(colorThresh, threshMat, Size(15,15), 7);
    findContours(threshMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point>> hull(contours.size());
    vector<RotatedRect> minRect(contours.size());

    // Get polygonal contours and bounding boxes
    Mat drawing;
    src.copyTo(drawing);
    vector<pair<int, double>> areas;

    // Find all hull contours
    for(unsigned int i = 0; i < contours.size(); ++i){
        convexHull(Mat(contours[i]), hull[i], false);
        areas.push_back(make_pair(i, contourArea(hull[i])));
    }

    sort(begin(areas), end(areas), [](pair<int,double> i, pair<int,double> j) { return i.second > j.second; });

    // Draw largest contours
    Point subcenter(0);
    for(unsigned int i = 0; i < areas.size() && (int)i < num_objects; ++i){
        int index = areas[i].first;
        drawContours(drawing, hull, index, Scalar(255,0,0), 2);
        // Draw minimum rectangles
        minRect[index] = minAreaRect(hull[index]);
        Point2f rect_points[4]; minRect[index].points(rect_points);
        for(unsigned int j = 0; j < 4; ++j){
            line(drawing, rect_points[j], rect_points[(j+1) % 4], Scalar(255,0,0), 2);
        }
        // Calculate moments
        Moments hull_moment = moments(hull[index]);
        int myx = hull_moment.m10 / hull_moment.m00;
        int myy = hull_moment.m01 / hull_moment.m00;
        subcenter += Point(myx, myy);
    }

    // Detect outliers and calculate center
    if(areas.size() != 0 && num_objects != 0){
        unsigned int divisor = min((int)areas.size(), num_objects);
        if(divisor != 0){
            center = Point(subcenter.x / divisor, subcenter.y / divisor);
            circle(drawing, center, 2, Scalar(0,0,255));
        }
    }

    drawing.copyTo(ROI);

    // Check if ROI exists
    if(!ROI.empty()){
        return true;
    }
    return false;
}

// enginePower takes a point and gives power a direction
// and magnitude
void enginePower(const Point &point, int &power, const unsigned int length, const unsigned int width){
    const int reflength = length / 2, range = 20, exponent = 1.5;
    int premod = 0;
    const double b = 100 / pow(abs(reflength) + range, exponent);
    // Process if in range
    if(point.x >= (reflength + range) || point.x <= (reflength - range)){
        premod = point.x - reflength;
    }
    power = b * pow(premod, exponent);
}

// extractRGBROI ...
bool extractRGBROI(Mat &src, Mat &ROI, Scalar lb, Scalar ub){
    Mat colorThresh, threshMat;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    Mat HSVMat;
    cvtColor(src, HSVMat, COLOR_BGR2GRAY);

    inRange(HSVMat, lb, ub, colorThresh);
    // Threshold image to find the most likely person
    morphologyEx(colorThresh, colorThresh, MORPH_GRADIENT, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
    GaussianBlur(colorThresh, colorThresh, Size(15,15), 7);
    adaptiveThreshold(colorThresh, threshMat, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 15, -5);
    findContours(colorThresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point>> hull(contours.size());
    vector<vector<Point>> contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());

    // Get polygonal contours and bounding boxes
    vector<pair<int, double>> bigvex;
    for(unsigned int i = 0; i< contours.size(); ++i){
        convexHull(Mat(contours[i]), hull[i], false);
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, false);
        boundRect[i] = boundingRect( Mat(contours_poly[i]));

        // Collect list of areas
        double localArea = contourArea(contours_poly[i]);
        bigvex.push_back(make_pair(i, localArea));
    }

    // Sort list of areas
    sort(begin(bigvex), end(bigvex), [](pair<int,double> i, pair<int,double> j) { return i.second > j.second; });

    // Create ROI from largest two areas
    for(unsigned int i = 0; i < bigvex.size() / 3; ++i){
        if(bigvex.size() >= 2){
            Point upperleft = (boundRect[bigvex.at(i).first].tl());
            Point bottomright = (boundRect[bigvex.at(i).first].br());
            Rect ROIrect = Rect(upperleft, bottomright);
            ROI = src(ROIrect);
            break;
        }
    }

    // Check if ROI exists
    if(!ROI.empty()){
        return true;
    }
    return false;
}

// extractDepthROI takes a depth source image, finds a likely person
// and returns a ROI
bool extractDepthROI(Mat &depthsrc, Mat &ROI, int &thresh){
    // Mat
    Mat threshMat;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // Threshold image to find the most likely person
    GaussianBlur(depthsrc, depthsrc, Size(15,15), 7);
    threshold(depthsrc, threshMat, thresh, 255, THRESH_BINARY);
    adaptiveThreshold(threshMat, threshMat, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 15, -5);
    findContours(threshMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point>> hull(contours.size());
    vector<vector<Point>> contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());

    // Get polygonal contours and bounding boxes
    vector<pair<int, double>> bigvex;
    for(unsigned int i = 0; i< contours.size(); ++i){
        convexHull(Mat(contours[i]), hull[i], false);
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, false);
        boundRect[i] = boundingRect( Mat(contours_poly[i]));

        // Collect list of areas
        double localArea = contourArea(contours_poly[i]);
        bigvex.push_back(make_pair(i, localArea));
    }

    // Sort list of areas
    sort(begin(bigvex), end(bigvex), [](pair<int,double> i, pair<int,double> j) { return i.second > j.second; });

    // Create ROI from largest two areas
    for(unsigned int i = 0; i < bigvex.size() / 3; ++i){
        if(bigvex.size() >= 2){
            Point upperleft = (boundRect[bigvex.at(i).first].tl() + boundRect[bigvex.at(i + 1).first].tl());
            Point ul = Point(upperleft.x / 2, upperleft.y / 2);
            Point bottomright = (boundRect[bigvex.at(i).first].br() + boundRect[bigvex.at(i + 1).first].br());
            Point lr = Point(bottomright.x / 2, bottomright.y / 2);
            Rect ROIrect = Rect(ul, lr);
            ROI = depthsrc(ROIrect);
            break;
        }
    }

    // Check if ROI exists
    if(!ROI.empty()){
        return true;
    }
    return false;
}

// differentiateObjects takes an image, extracts a range of colors
// (HSV lower and upper bounds), and differentiates the objects of
// that color from each other.
void differentiateObjects(Mat &src, Mat &dst, int &thresh, Scalar HSVlb, Scalar HSVub){
    Mat imgHSV;
    dst.setTo(Scalar(0,0,0));
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
