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
const double MHI_DURATION = 0.2;
const double MAX_TIME_DELTA = 0.5;
const double MIN_TIME_DELTA = 0.05;
// number of cyclic frame buffer used for motion detection
// (should, probably, depend on FPS)
const int N = 4;



// temporary images
Mat mhi; // MHI
Mat orient; // orientation
Mat mask; // valid orientation mask

Mat segmask; // motion segmentation map
vector<Rect> storage; // temporary storage
bool init = false;
// parameters:
//  img - input video frame
//  dst - resultant motion picture
//  args - optional parameters
static void  update_mhi( Mat &img1, Mat& img2, Mat &dst, int diff_threshold )
{
    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
//    CvSize size = cvSize(img->width,img->height); // get current frame size
    Mat silh;
    vector<CvRect> seq;
    CvRect comp_rect;
    double count;
    double angle;
    CvPoint center;
    double magnitude;
    CvScalar color;

    Mat i1, i2;
    cvtColor( img1, i1, CV_BGR2GRAY ); // convert frame to grayscale
    cvtColor( img2, i2, CV_BGR2GRAY ); // convert frame to grayscale

    absdiff(i1, i2, silh); // get difference between frames
//    cout << "here absdiff Done  " << idx1 << " " << idx2 << " "  << last << endl;
    dst = silh;
//    threshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
//    cout << "here threshold Done" << endl;
//    dst = silh;
//    updateMotionHistory( silh, mhi, timestamp, MHI_DURATION ); // update MHI
//    cout << "here updateMotionHistory Done" << endl;

    // convert MHI to blue 8u image
//    cvCvtScale( mhi, mask, 255./MHI_DURATION,
//                (MHI_DURATION - timestamp)*255./MHI_DURATION );
//    mhi.convertTo(mask,CV_8U, 255./MHI_DURATION,
//                (MHI_DURATION - timestamp)*255./MHI_DURATION );
//    dst.zeros()
//    vector<Mat> merge_vect(4,Mat::zeros(mask.rows, mask.cols,CV_8U));
//    merge_vect[0] = mask;
//    cv::merge( merge_vect, dst );

    // calculate motion gradient orientation and valid orientation mask
//    calcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );


    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
//    segmentMotion( mhi, segmask, storage, timestamp, MAX_TIME_DELTA );

    // iterate through the motion components,
    // One more iteration (i == -1) corresponds to the whole image (global motion)
//    for( i = -1; i < seq->total; i++ ) {

//        if( i < 0 ) { // case of the whole image
//            comp_rect = cvRect( 0, 0, size.width, size.height );
//            color = CV_RGB(255,255,255);
//            magnitude = 100;
//        }
//        else { // i-th motion component
//            comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;
//            if( comp_rect.width + comp_rect.height < 100 ) // reject very small components
//                continue;
//            color = CV_RGB(255,0,0);
//            magnitude = 30;
//        }

        // select component ROI
        //        cvSetImageROI( silh, comp_rect );
        //        cvSetImageROI( mhi, comp_rect );
        //        cvSetImageROI( orient, comp_rect );
        //        cvSetImageROI( mask, comp_rect );


        // calculate orientation
        //        angle = cvCalcGlobalOrientation( orient, mask, mhi, timestamp, MHI_DURATION);
        //        angle = 360.0 - angle;  // adjust for images with top-left origin

        //        count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

        //        cvResetImageROI( mhi );
        //        cvResetImageROI( orient );
        //        cvResetImageROI( mask );
        //        cvResetImageROI( silh );

//        cvRectangle(img,cvPoint(comp_rect.x,comp_rect.y),
//                    cvPoint(comp_rect.x+comp_rect.width,comp_rect.y+ comp_rect.height),cvScalar(255,0,0));

//        // check for the case of little motion
//        if( count < comp_rect.width*comp_rect.height * 0.05 )
//            continue;

//        cvRectangle(img,cvPoint(comp_rect.x,comp_rect.y),
//                    cvPoint(comp_rect.x+comp_rect.width,comp_rect.y+ comp_rect.height),cvScalar(255,0,0));


        // draw a clock with arrow indicating the direction
        //        center = cvPoint( (comp_rect.x + comp_rect.width/2),
        //                          (comp_rect.y + comp_rect.height/2) );

        //        cvCircle( dst, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
        //        cvLine( dst, center, cvPoint( cvRound( center.x + magnitude*cos(angle*CV_PI/180)),
        //                cvRound( center.y - magnitude*sin(angle*CV_PI/180))), color, 3, CV_AA, 0 );
//    }
}


int main(int argc, char** argv)
{
    // ring image buffer
//    vector<Mat> buf(N);
//    int last = 0, idx1, idx2;
    Mat motion, image, image_prev;
    vector<Mat> buf(4);
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

//    cvNamedWindow( "Motion", 1 );
    cvNamedWindow( "Image", 1 );

    cap >> image_prev;


    for(;;)
    {
        cap >> image;
        if( image.empty() )
            break;
//        idx2 = (last + 1) % N; // index of (last - (N-1))th frame
//        last = idx2;
        motion = image;
//        update_mhi( image_prev, image , motion, 30 );

        Mat i1, i2;
        cvtColor( image_prev, i1, CV_BGR2GRAY ); // convert frame to grayscale
        cvtColor( image, i2, CV_BGR2GRAY ); // convert frame to grayscale
        absdiff(i1,i2,motion);
        imshow("Motion", motion );
        imshow( "Image", image );

        if( cvWaitKey(30) >= 0 )
            break;
        image.copyTo(image_prev);
    }
    cap.release();
//    cvReleaseCapture( &capture );
    cvDestroyWindow( "Motion" );
    cvDestroyWindow( "Image" );


    return 0;
}

#ifdef _EiC
main(1,"motempl.c");
#endif
