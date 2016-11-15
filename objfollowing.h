#ifndef OBJFOLLOWING_H
#define OBJFOLLOWING_H

#include <map>

#include "opencv2/core.hpp"

using namespace std;
using namespace cv;

// State enumerations
enum MOTION_STATE {STATIONARY = 0, MOVING = 1};
enum TRACK_STATE {FINDING = 0, TRACKING = 1};

// Color ...
extern map<string,int> HSV;

// Functions
void posterizeRGB(Mat &src, Mat &dst);
bool extractRGBROI(Mat &src, Mat &ROI, Scalar lb, Scalar ub);
bool extractDepthROI(Mat &depthsrc, Mat &ROI, int &thresh);
void buildContours(Mat &src, Mat &dst, int &thresh);
void differentiateObjects(Mat &src, Mat &ROI, int &thresh, Scalar HSVlb, Scalar HSVub);

#endif // OBJFOLLOWING_H
