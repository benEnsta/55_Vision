#include "mykalman.h"


myKalman::myKalman()
{
    KalmanFilter KF(4, 2, 0);
    Mat_<float> state(4, 1); /* (x, y, Vx, Vy) */
    Mat processNoise(4, 1, CV_32F);
    Mat_<float> measurement(2,1);
    measurement.setTo(Scalar(0));

    KFs = KF;
    measurements = measurement;
}

void myKalman::setKalman(float x, float y)
{
    KFs.statePre.at<float>(0) = x;
    KFs.statePre.at<float>(1) = y;
    KFs.statePre.at<float>(2) = 0;
    KFs.statePre.at<float>(3) = 0;
    KFs.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);

    setIdentity(KFs.measurementMatrix);
    setIdentity(KFs.processNoiseCov, Scalar::all(1e-2));
    setIdentity(KFs.measurementNoiseCov, Scalar::all(1e-3));
    setIdentity(KFs.errorCovPost, Scalar::all(10));


}

Point myKalman::step1()
{
    Mat prediction = KFs.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
    return predictPt;
}

Point myKalman::step2()
{
    Mat estimated = KFs.correct(measurements);
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    return statePt;
}

void myKalman::changeMeasure(int x, int y)
{
    measurements(0) = x;
    measurements(1) = y;
}
