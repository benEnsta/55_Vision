#ifndef DETECTEDOBJECT_H
#define DETECTEDOBJECT_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"



using namespace cv;
using namespace std;
class detectedObject
{
public:
    detectedObject();
    void setKalman(float x, float y);
    Point step1();
    Point step2();
    void changeMeasure(int x,int y);

    Rect rect;
    Point2f pos, old_pos;


private:
    KalmanFilter KFs;
    Mat_<float> measurements;
};
#endif // DETECTEDOBJECT_H
