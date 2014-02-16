#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>


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

int test_detector(Mat &img1, Mat &img2){
    if(img1.empty() || img2.empty())
    {
        printf("Can't read one of the images\n");
        return -1;
    }

    // detecting keypoints
    SurfFeatureDetector detector(400);
    vector<KeyPoint> keypoints1, keypoints2;
    detector.detect(img1, keypoints1);
    detector.detect(img2, keypoints2);

    // computing descriptors
    SurfDescriptorExtractor extractor;
    Mat descriptors1, descriptors2;
    extractor.compute(img1, keypoints1, descriptors1);
    extractor.compute(img2, keypoints2, descriptors2);

    // matching descriptors
    BruteForceMatcher<L2<float> > matcher;
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // drawing the results
    namedWindow("matches", 0);
    Mat img_matches;
    drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
    imshow("matches", img_matches);
    //    waitKey(0);
}


Moments extract_shape2(Mat& src_gray){

    const int MAX_COUNT = 15;

    vector<Point2f> points;
    int dilation_size =1;
    Mat element = getStructuringElement( MORPH_RECT,
                                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           Point( -1, -1) );
//    // automatic initialization
    threshold( src_gray, src_gray, 0, 255, THRESH_OTSU);
    morphologyEx(src_gray,src_gray,MORPH_GRADIENT,element,Point(-1,-1),1);
    morphologyEx(src_gray,src_gray,MORPH_CLOSE,element,Point(-1,-1),1);

    imshow("contour",src_gray);
    return moments(src_gray,true);

}

int main(int argc, char** argv)
{

    VideoCapture cap;
    MotionDetector detector;

    Mat image_prev, image;
    Mat gray, prevGray;
    Mat img1, img2;

    // Kalman
    KalmanFilter KF(4, 2, 0);
    Mat_<float> state(4, 1); /* (x, y, Vx, Vy) */
    Mat processNoise(4, 1, CV_32F);
    Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));


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
    cvNamedWindow("contour",0);

    for(int ind = 0;;ind++)
    {
        Mat frame;
        cap >> frame;
        if( frame.empty() )
            break;

        frame.copyTo(image);


        vector<Rect> roi = detector.update(image, 30);

        for(uint i = 0; i < roi.size(); i++){
            rectangle(image,roi[0],cvScalar(255,0,0));
        }
        //        if(roi.size() > 0 && roi[0].width+roi[0].height < 500){
        //            img1 = image(roi[0]);
        //            img2 = image_prev(roi[0]);
        //            test_detector(img1,img2);
        //        }

        if(roi.size() > 0 && roi[0].width+roi[0].height < 500){

            cvtColor(detector.getMotion()(roi[0]),img2,CV_BGR2GRAY);
            cvtColor(image(roi[0]),img1,CV_BGR2GRAY);
//            cout << img1.type() << " " << img2.type() << endl;
            Mat img3 = img1 & img2;
            Moments mu = extract_shape2(img3);
            Point2f center = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00) + Point2f(roi[0].x, roi[0].y);
            cout << center << " "<<roi[0] << " " << roi[0].contains(center) << endl;


            circle(image,center,3,cvScalar(0,0,255),3);


        }



        imshow("Motion", detector.getMotion() );
        imshow( "Image", image );

//        cvWaitKey(0);
        if( cvWaitKey(30) >= 0 )
            break;

        image.copyTo(image_prev);


    }
    cap.release();

    cvDestroyWindow( "Motion" );
    cvDestroyWindow( "Image" );



    return 0;
}




#ifdef _EiC
main(1,"motempl.c");
#endif
