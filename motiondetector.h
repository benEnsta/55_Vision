#ifndef MOTIONDETECTOR_H
#define MOTIONDETECTOR_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"

#include <time.h>
#include <stdio.h>
#include <ctype.h>

#include <iostream>

using namespace cv;
using namespace std;

class MotionDetector
{
public:
    MotionDetector(int buffer_depth = 4, double mhi_duration = 0.3,double max_time_delta= 0.2, double min_time_delta = 0.01);

    vector<Rect> update(Mat &img, Mat &dst, int diff_threshold);


    vector<Rect> compute(Mat &img1, Mat &img2, Mat &dst, int diff_threshold);
private:
    // various tracking parameters (in seconds)
    const double MHI_DURATION;// = 0.3;
    const double MAX_TIME_DELTA;// = 0.2;
    const double MIN_TIME_DELTA;// = 0.01;
    // number of cyclic frame buffer used for motion detection
    // (should, probably, depend on FPS)
    const int N;// = 3;

    // ring image buffer of size N
    vector<Mat> buffer;//(N);
    int last;// = 0;

    // temporary images
    Mat mhi; // MHI
    Mat silh,silh1;
    Mat orient; // orientation
    Mat mask; // valid orientation mask
    Mat segmask; // motion segmentation map

    vector<Rect> seq;


    void resize_matrix(int rows, int cols);


};

#endif // MOTIONDETECTOR_H
