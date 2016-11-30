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
bool discernObject(Mat &src, const int &num_objects, Point &center, Point &prevPoint, Rect &boundRect, const Scalar lb, const Scalar ub);
void forwardPower(Mat &depthsrc, Rect &binding);

#endif // OBJFOLLOWING_H
