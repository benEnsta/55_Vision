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
//const int N = 3;

// ring image buffer
//int last = 0;
//vector<Mat> buf(3);

int test_detector(Mat &img1, Mat &img2){

    //Mat img1 = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    //Mat img2 = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
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

const int MAX_COUNT = 500;

Point2f point;
vector<Point2f> points[2];

TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
Size subPixWinSize(10,10), winSize(31,31);

Mat gray, prevGray;

int test_opticalFlow(Mat & image, bool needToInit){

    cvtColor(image, gray, COLOR_BGR2GRAY);

    if( needToInit )
    {
        // automatic initialization
        goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
        cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
    }
    else if( !points[0].empty() )
    {
        vector<uchar> status;
        vector<float> err;
        if(prevGray.empty())
            gray.copyTo(prevGray);
        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                3, termcrit, 0, 0.001);
        size_t i, k;
        for( i = k = 0; i < points[1].size(); i++ )
        {
            if( !status[i] )
                continue;

            points[1][k++] = points[1][i];
            circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
        }
        points[1].resize(k);
    }

    if(points[1].size() < (size_t)MAX_COUNT )
    {
        vector<Point2f> tmp;
        tmp.push_back(point);
        cornerSubPix( gray, tmp, winSize, cvSize(-1,-1), termcrit);
        points[1].push_back(tmp[0]);

    }

    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);

}

void extract_shape(Mat& img){
    Mat src_gray;
    RNG rng(12345);
    int dilation_size =1;
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           Point( -1, -1) );


    cvtColor(img,src_gray,CV_BGR2GRAY);

    /// Apply the dilation operation



//    blur( src_gray, src_gray, Size(3,3) );

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    int thresh = 30;
    /// Detect edges using canny
    threshold( src_gray, canny_output, thresh, 255, THRESH_OTSU );
//    Canny(src_gray,canny_output,thresh,1.5*thresh,3);
    erode(canny_output, canny_output, element,Point(-1,-1), 2 );
    dilate( canny_output, canny_output, element,Point(-1,-1), 2 );

    dilate( canny_output, canny_output, element,Point(-1,-1), 2 );
    erode(canny_output, canny_output, element,Point(-1,-1), 2 );

    /// Find contours
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }


    /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//        drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
//        circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
    }
    imshow("contour",drawing);

}

Point2f extract_shape2(Mat& src_gray){

    const int MAX_COUNT = 50;

    vector<Point2f> points;


    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);

//    Mat src_gray;
//    cvtColor(img,src_gray,CV_BGR2GRAY);

//    // automatic initialization
    goodFeaturesToTrack(src_gray, points, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
//    if(points.size() > 0)
//        cornerSubPix(src_gray, points, subPixWinSize, Size(-1,-1), termcrit);


    for( uint i = 0; i < points.size(); i++ )
    {
        //points[1][k++] = points[1][i];
        circle( src_gray, points[i], 3, Scalar(255,255,0), -1, 8);
    }




    imshow("contour",src_gray);
    return mc(points);

}

int main(int argc, char** argv)
{

    VideoCapture cap;
    MotionDetector detector;

    Mat image_prev, image;
    Mat gray, prevGray;
    Mat img1, img2;

    ;
    bool needToInit = false;
    bool nightMode = false;

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
        cout << roi.size() << endl;

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
            Rect rect = extract_shape2(img3);
            rect.x += roi[0].x;
            rect.y += roi[0].y;
            rectangle(image,rect,cvScalar(0,0,255));

        }


        //        if( ind == 1) test_opticalFlow(image,true);
        //        else {
        //          test_opticalFlow(image,false);
        //        }

        imshow("Motion", detector.getMotion() );
        imshow( "Image", image );

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
