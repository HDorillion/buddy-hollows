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
bool discernObject(Mat &src, const Scalar lb, const Scalar ub, const int &num_objects, Point &center, Mat &ROI);
void enginePower(const Point &point, int &power, const unsigned int length = 640, const unsigned int width = 480);
bool extractRGBROI(Mat &src, Mat &ROI, Scalar lb, Scalar ub);
bool extractDepthROI(Mat &depthsrc, Mat &ROI, int &thresh);
void differentiateObjects(Mat &src, Mat &ROI, int &thresh, Scalar HSVlb, Scalar HSVub);

#endif // OBJFOLLOWING_H
