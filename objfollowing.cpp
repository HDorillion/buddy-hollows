#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"

#include "objfollowing.h"

// Map of HSV values
map<string,int> HSV {
    {"Hl", 100},
    {"Hh", 120},
    {"Sl", 100},
    {"Sh", 255},
    {"Vl", 50},
    {"Vh", 180}
};

// Color variable range
RNG rng(12345);

// posterizeRGB takes a source image and creates a posterized version
void posterizeRGB(Mat &src, Mat &dst){
    Mat samples(src.rows * src.cols, 3, CV_32F);
    for( int y = 0; y < src.rows; y++ )
        for( int x = 0; x < src.cols; x++ )
            for( int z = 0; z < 3; z++)
                samples.at<float>(y + x*src.rows, z) = src.at<Vec3b>(y,x)[z];

    int clusterCount = 10;
    Mat labels;
    int attempts = 2;
    Mat centers;
    kmeans(samples, clusterCount, labels,
           TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0),
           attempts, KMEANS_PP_CENTERS, centers );

    Mat new_image( src.size(), src.type() );
    for( int y = 0; y < src.rows; y++ ){
        for( int x = 0; x < src.cols; x++ ){
            int cluster_idx = labels.at<int>(y + x*src.rows,0);
            new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
            new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
            new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
        }
    }

    new_image.copyTo(dst);
}

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
//    threshold(colorThresh, threshMat, thresh, 255, THRESH_BINARY);
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
            Point upperleft = (boundRect[bigvex.at(i).first].tl() + boundRect[bigvex.at(i + 1).first].tl()) / 2;
            Point bottomright = (boundRect[bigvex.at(i).first].br() + boundRect[bigvex.at(i + 1).first].br()) / 2;
            Rect ROIrect = Rect(upperleft, bottomright);
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

// differentiateObjects takes an image, extracts a range of colors
// (HSV lower and upper bounds), and differentiates the objects of
// that color from each other.
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
