#include "motiondetector.h"

MotionDetector::MotionDetector(int buffer_depth, double mhi_duration,
                               double max_time_delta, double min_time_delta) :
    MHI_DURATION(mhi_duration),
    MAX_TIME_DELTA(max_time_delta),
    MIN_TIME_DELTA(min_time_delta),
    N(buffer_depth),
    buf(buffer_depth)
{

    cout << "Initialisation du buffer tournant size: "<< buf.size() << endl;
    for(uint i = 0; i < N; i++){
        buf[i].create(100,100,CV_8UC1);
    }
    mhi  = Mat::zeros(100,100,CV_32FC1);
}


void MotionDetector::detect(Mat &in_img, Mat &out_dst, int diff_threshold){


}
