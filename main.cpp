#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"

#include <time.h>
#include <stdio.h>
#include <ctype.h>

#include <iostream>

using namespace cv;
using namespace std;

static void help(void)
{
    printf(
                "\nThis program demonstrated the use of motion templates -- basically using the gradients\n"
                "of thresholded layers of decaying frame differencing. New movements are stamped on top with floating system\n"
                "time code and motions too old are thresholded away. This is the 'motion history file'. The program reads from the camera of your choice or from\n"
                "a file. Gradients of motion history are used to detect direction of motoin etc\n"
                "Usage :\n"
                "./motempl [camera number 0-n or file name, default is camera 0]\n"
                );
}
// various tracking parameters (in seconds)
const double MHI_DURATION = 0.3;
const double MAX_TIME_DELTA = 0.2;
const double MIN_TIME_DELTA = 0.01;
// number of cyclic frame buffer used for motion detection
// (should, probably, depend on FPS)
const int N = 3;

// ring image buffer
vector<Mat> buf(N);
int last = 0;

// temporary images
Mat mhi; // MHI
Mat orient; // orientation
Mat mask; // valid orientation mask

Mat segmask; // motion segmentation map

bool init = false;
// parameters:
//  img - input video frame
//  dst - resultant motion picture
//  args - optional parameters

static void  update_mhi( Mat &img, Mat &dst, int diff_threshold )
{
    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds

    int i, idx1 = last, idx2;
    Mat silh, silh1;
    vector<Rect> seq;
    CvScalar color;

    if( init == false ) {

        cout << "here init" << endl;
        buf =  vector<Mat>(N);
        for(uint i = 0; i < N; i++){
            buf[i].create(img.rows, img.cols,CV_8UC1);
        }
        mhi  = Mat::zeros(img.rows,img.cols,CV_32FC1);
//        silh = Mat::zeros(img.rows, img.cols,CV_8UC1);
        init = true;
    }

    // put image inside the framebuffer
    cvtColor( img, buf[last], CV_BGR2GRAY ); // convert frame to grayscale

    // update indices
    idx2 = (last + 1) % N; // index of (last - (N-1))th frame
    last = idx2;



    absdiff(buf.at(idx1), buf.at(idx2), silh ); // get difference between frames


    threshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
    medianBlur(silh,silh1,3);
    updateMotionHistory( silh1, mhi, timestamp, MHI_DURATION ); // update MHI

    // convert MHI to blue 8u image
    mhi.convertTo(mask,CV_8U, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );

    vector<Mat> merge_vect(4,Mat::zeros(mask.rows, mask.cols,CV_8U));
    merge_vect[0] = mask;
    cv::merge( merge_vect, dst );

    // calculate motion gradient orientation and valid orientation mask
    calcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );


    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    segmentMotion( mhi, segmask, seq, timestamp, MAX_TIME_DELTA );

    for( i = 0; i < seq.size(); i++ ){
        Rect comp_rect = seq.at(i);

        if( comp_rect.width + comp_rect.height < 100 ) // reject very small components
            continue;
        color = CV_RGB(255,0,0);
        rectangle(img,comp_rect,color);
    }
    seq.clear();
}


int main(int argc, char** argv)
{
    Mat motion, image;
    VideoCapture cap;

    help();

    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        cap.open(argc == 2 ? argv[1][0] - '0' : 0);
    else if( argc == 2 )
        cap.open(argv[1]);
    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

    cvNamedWindow( "Motion", 1 );
    cvNamedWindow( "Image", 1 );

    for(;;)
    {
        cap >> image;
        if( image.empty() )
            break;
        motion = image;
        update_mhi( image, motion, 30 );
        imshow("Motion", motion );
        imshow( "Image", image );

        if( cvWaitKey(30) >= 0 )
            break;
    }
    cap.release();

    cvDestroyWindow( "Motion" );
    cvDestroyWindow( "Image" );



    return 0;
}

#ifdef _EiC
main(1,"motempl.c");
#endif
