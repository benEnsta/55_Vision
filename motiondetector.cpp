#include "motiondetector.h"

MotionDetector::MotionDetector(int buffer_depth, double mhi_duration,
                               double max_time_delta, double min_time_delta) :
    MHI_DURATION(mhi_duration),
    MAX_TIME_DELTA(max_time_delta),
    MIN_TIME_DELTA(min_time_delta),
    N(buffer_depth),
    buffer(buffer_depth),
    last(0)
{

    cout << "Initialisation du buffer tournant size: "<< buffer.size() << endl;
    for(uint i = 0; i < buffer.size(); i++){
        buffer[i].create(0,0,CV_8UC1);
    }
    mhi  = Mat::zeros(0,0,CV_32FC1);
}


void MotionDetector::resize_matrix(int rows, int cols ){

    cout << "resize " << endl;

    for(uint i = 0; i < buffer.size(); i++){
        buffer[i].create(rows,cols,CV_8UC1);

    }
    mhi  = Mat::zeros(rows,cols,CV_32FC1);

}

vector<Rect> MotionDetector::compute(Mat &img1, Mat &img2, int diff_threshold){

    assert((img1.type() == CV_8UC1) && (img2.type() == CV_8UC1) );

    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
    vector<Rect> seq;
    roi.clear();

    absdiff(img1, img2, silh ); // get difference between frames


    threshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
    medianBlur(silh,silh1,3);
    updateMotionHistory( silh1, mhi, timestamp, MHI_DURATION ); // update MHI

    // convert MHI to blue 8u image
    mhi.convertTo(mask,CV_8U, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );

    vector<Mat> merge_vect(4,Mat::zeros(mask.rows, mask.cols,CV_8U));
    merge_vect[0] = mask;
    cv::merge( merge_vect, motion );

    // calculate motion gradient orientation and valid orientation mask
    calcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );


    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    segmentMotion( mhi, segmask, seq, timestamp, MAX_TIME_DELTA );
    vector<Rect> result;
    for(uint i = 0; i < seq.size(); i++ ){
        Rect comp_rect = seq.at(i);

        if( comp_rect.width + comp_rect.height < 100 ) // reject very small components
            continue;
//        color = CV_RGB(255,0,0);
//        rectangle(img,comp_rect,color);
        roi.push_back(comp_rect);
    }

    for(uint i = 0; i < roi.size(); i++){
        for(uint j = i+1; j < roi.size(); j++){
            Rect tmp = roi[i] & roi[j];
            if(tmp.area() > 0){
                roi[i] |= roi[j];
                roi[j] = roi[i];
            }
        }
    }



    return roi;
}

Mat& MotionDetector::getFirst()
{
    return buffer[idx1];
}

Mat& MotionDetector::getSecond()
{
    return buffer[idx2];
}

Mat& MotionDetector::getMotion(){
    return motion;
}

Mat &MotionDetector::getSilh()
{
    return this->silh1;
}

vector<Rect> MotionDetector::update(Mat &img, int diff_threshold){

    idx1 = last;

    if(img.rows != mhi.rows || img.cols != mhi.cols)
        resize_matrix(img.rows,img.cols);

    // put image inside the framebuffer
    cvtColor( img, buffer[last], CV_BGR2GRAY ); // convert frame to grayscale

    // update indices
    idx2 = (last + 1) % N; // index of (last - (N-1))th frame
    last = idx2;


    return compute(getFirst(),getSecond(),diff_threshold);


}
