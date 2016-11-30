#include <cmath>
#include <iostream>

#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"

#include "objfollowing.h"
#include "MovingAverages.h"

// Map of HSV values
map<string,int> HSV {
    {"Hl", 35},
    {"Hh", 35},
    {"Sl", 100},
    {"Sh", 255},
    {"Vl", 100},
    {"Vh", 255}
};

// Color variable range
RNG rng(12345);

// discernObject
bool discernObject(Mat &src, const int &num_objects, Point &center, Point &prev_center, Rect &boundRect, const Scalar lb, const Scalar ub){
    Mat colorThresh, threshMat;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    Mat HSVMat;
    cvtColor(src, HSVMat, COLOR_BGR2HSV);
    bool ctrisempty = (center.x == 0 && center.y == 0);
//    double mags = 0, magsquared = 0;

    // Find contours within color range
    inRange(HSVMat, lb, ub, colorThresh);
    GaussianBlur(colorThresh, threshMat, Size(15,15), 7);
    findContours(threshMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point>> hull(contours.size());
    vector<Point> wholehull;
//    vector<RotatedRect> minRect(contours.size());

    // Get polygonal contours and bounding boxes
    Mat drawing;
    src.copyTo(drawing);
    vector<pair<int, double>> areas;
    unsigned int divisor = 0;

    // Find all hull contours within size range
    for(unsigned int i = 0; i < contours.size(); ++i){
        convexHull(Mat(contours[i]), hull[i], false);
        double temp_area = contourArea(hull[i]);
        if(temp_area >= 1000){
            areas.push_back(make_pair(i, temp_area));
        }
    }

    sort(begin(areas), end(areas), [](pair<int,double> i, pair<int,double> j) { return i.second > j.second; });

    // Draw largest contours
    Point subcenter(0);
    for(unsigned int i = 0; i < areas.size() && (int)i < num_objects; ++i){
        int index = areas[i].first;
        drawContours(drawing, hull, index, Scalar(255,0,0), 2);
        wholehull.insert(wholehull.end(), hull[index].begin(), hull[index].end());
        // Draw minimum rectangles
//        minRect[index] = minAreaRect(hull[index]);
//        Point2f rect_points[4]; minRect[index].points(rect_points);
//        for(unsigned int j = 0; j < 4; ++j){
//            line(drawing, rect_points[j], rect_points[(j+1) % 4], Scalar(255,0,0), 2);
//        }
        // Calculate moments and stage centroid
        Moments hull_moment = moments(hull[index]);
        int myx = hull_moment.m10 / hull_moment.m00;
        int myy = hull_moment.m01 / hull_moment.m00;
        Point test_point(myx, myy);
        MovingAverages::addXValue(myx, hull_moment.m00);

        // If no center
        if(ctrisempty){
            center = subcenter = test_point;
            ctrisempty = false;
            ++divisor;
            continue;
        }
        else{
            // Test for averaging
            if(subcenter == Point(0)) subcenter = test_point;
            else subcenter = Point((subcenter.x + test_point.x) / 2, (subcenter.y + test_point.y) / 2);
            ++divisor;
        }
    }

    // Calculate center and new standard deviation
    if(divisor != 0){
        center = subcenter;
        circle(drawing, center, 2, Scalar(0,0,255));
        prev_center = center;
    }

    boundRect = boundingRect(wholehull);
    rectangle(drawing, boundRect, Scalar(255,0,0), 2);
    drawing.copyTo(src);

    // Check if ROI exists
    if(!src.empty()){
        return true;
    }
    return false;
}

// forwardPower takes a depth source image, means the object, localized,
// and calls to set the driving buffer
void forwardPower(Mat &depthsrc, Rect &binding){
    Rect reduced = Rect(Point(
                            (binding.tl().x + binding.width) * 2 / 3, (binding.tl().y + binding.height) * 2 / 3),
                        Point(
                            (binding.tl().x + binding.width) * 1 / 3, (binding.tl().y + binding.height) * 1 / 3));
//    Scalar tempMean = mean(depthsrc(reduced));
    MovingAverages::addXValue((binding.tl().x + binding.width) / 2, mean(depthsrc(reduced))[0]);
}
