#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/opencv.hpp>

#include<iostream>
#include<vector>

#include <time.h>
#include <stdio.h>
#include <ctype.h>

#include <iostream>

#include "motiondetector.h"
#include "detectedObject.h"

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


Moments extract_shape2(Mat &img, Mat& src_gray, Rect roi){

    const int MAX_COUNT = 15;

    vector<Point2f> points;
    int dilation_size =1;
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( -1, -1) );
    //    // automatic initialization
    //        threshold( src_gray, src_gray, 10, 255, THRESH_OTSU);
    //    morphologyEx(src_gray,src_gray,MORPH_GRADIENT,element,Point(-1,-1),1);
    //    morphologyEx(src_gray,src_gray,MORPH_CLOSE,element,Point(-1,-1),1);

    vector<vector<Point> > contours;

    vector<Vec4i> hierarchy;

    findContours( src_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(roi.x, roi.y) );

    Rect box;
    for(uint i = 0; i < contours.size(); i++){
        if(i == 0)
            box = boundingRect(contours[i]);
        else
            box |= boundingRect(contours[i]);
    }
    rectangle(img,box,cvScalar(0,255,0),2);
    imshow("contour",src_gray);
    return moments(src_gray,true);

}



vector<Rect> extractSubRegion(Mat& fore,vector<Rect> roi){
    Mat tmp;
    vector<Rect> roi_red;
    for(uint i = 0; i < roi.size(); i++){
        if(roi[i].width+roi[i].height > 500)
            continue;
        fore(roi[i]).copyTo(tmp);

        medianBlur(tmp,tmp,3);
//        Canny(tmp,tmp,100,200);
        morphologyEx(tmp,tmp,MORPH_GRADIENT,Mat());

        vector<vector<Point> > contours;

//        vector<Vec4i> hierarchy;
        Mat tmp2;
        tmp.copyTo(tmp2);

        findContours( tmp2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(roi[i].x, roi[i].y) );

        Rect box;
        for(uint i = 0; i < contours.size(); i++){
            if(i == 0)
                box = boundingRect(contours[i]);
            else
                box |= boundingRect(contours[i]);
        }
        roi_red.push_back(box);
        imshow("contour",tmp);
    }

    return roi_red;
}


Mat image;
void on_mouse(int event, int x, int y, int flags, void* param) {
    static int i = 0;
    if (event == CV_EVENT_LBUTTONUP)
    {
//        last_mouse = mouse_info;
//        mouse_info.x = x;
//        mouse_info.y = y;
        circle(image,Point(x,y),3,cvScalar(255,0,0),2);
        imshow("Image", image);
        cout << "p_src[" << i << "] = Point2f(" << x << "," << y << ");" <<endl;
        i++;
    }
}

int main(int argc, char** argv)
{

    VideoCapture cap;
    MotionDetector detector;
    BackgroundSubtractorMOG2 bg(100,-1,false);

    Mat image_prev, frame;
    Mat gray, prevGray;
    Mat img1, img2;
    Mat back, fore;


    vector<detectedObject> KFs;



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
     setMouseCallback("Image", on_mouse, 0);
    cvNamedWindow("contour",0);
    cvNamedWindow("perspective",0);


    Point2f p_src[4];
//    p_src[0] = Point2f(26,367);
//    p_src[1] = Point2f(442,470);
//    p_src[2] = Point2f(470,209);
//    p_src[3] = Point2f(210,181);


    p_src[0] = Point2f(24,367);
    p_src[1] = Point2f(145,173);
    p_src[2] = Point2f(552,164);
    p_src[3] = Point2f(520,425);

    double w = 30;


    Point2f p_dst[4];
    p_dst[0] = (Point2f(2*w,0));
    p_dst[1] = (Point2f(0,7*w));
    p_dst[2] = (Point2f(11*w,10*w));
    p_dst[3] = (Point2f(10*w,1*w));


    Mat M = getPerspectiveTransform(p_src, p_dst);
    cout << M << endl;
    bool detected = false;

    //    KF.setKalman(0,0);
    for(int ind = 0;;ind++)
    {
        // Get image
        cap >> frame;
        if( frame.empty() )
            break;

        frame.copyTo(image);

        // Extract background/foregroung info
        bg(image,fore);
        bg.getBackgroundImage(back);


        Mat pers;// = Mat(300,300,image.type());
        warpPerspective(image,pers,M,Size(11*w,11*w));
        imshow("perspective",pers);

        vector<Rect> roi = detector.update(image, 30);

        vector<Rect> interret = extractSubRegion(fore,roi);


        for(uint i = 0; i < roi.size(); i++){
            if(roi[i].width+roi[i].height > 500)
                continue;
            rectangle(image,roi[i],cvScalar(255,0,0));
            rectangle(image,interret[i],cvScalar(0,255,0));


        }







        imshow("Motion", detector.getMotion() );
        imshow( "Image", image );

        cvWaitKey(0);
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
