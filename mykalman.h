#ifndef MYKALMAN_H
#define MYKALMAN_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"



using namespace cv;
using namespace std;
class myKalman
{
public:
    myKalman();
    void setKalman(float x, float y);
    Point step1();
    Point step2();
    void changeMeasure(int x,int y);

private:
    KalmanFilter KFs;
    Mat_<float> measurements;
};
#endif // MYKALMAN_H
