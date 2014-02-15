#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"

#include <time.h>
#include <stdio.h>
#include <ctype.h>

#include <iostream>

#include "motiondetector.h"

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
//const int N = 3;

// ring image buffer
//int last = 0;
//vector<Mat> buf(3);




int main(int argc, char** argv)
{
    Mat motion, image;
    VideoCapture cap;
    MotionDetector detector;
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
        detector.update(image, motion, 30 );
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
