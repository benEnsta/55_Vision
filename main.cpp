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



vector<Rect> extractSubRegion(Mat& fore,vector<Rect> &roi, vector<Point2f> &centre){
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
        Moments mu =  moments(tmp,true);
        centre.push_back(Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 )+Point2f(box.x,box.y ));

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

Mat computePerspectiveTransform(Point2f *p_src, Point2f *p_dst){

    double w = 30;

    p_src[0] = Point2f(74,401);
    p_src[1] = Point2f(205,240);
    p_src[2] = Point2f(477,270);
    p_src[3] = Point2f(497,476);

    p_dst[0] = (Point2f(5*w,0*w));
    p_dst[1] = (Point2f(5*w,6*w));
    p_dst[2] = (Point2f(12*w,6*w));
    p_dst[3] = (Point2f(12*w,0*w));

    return getPerspectiveTransform(p_src, p_dst);
}

//int track_object(vector<detectedObject> &KFs, Rect roi, Point2f &centre){
//    if(KFs.size() == 0){
//        KFs.push_back(detectedObject());
//        KFs[0].setKalman(0,0);
////        obj.setKalman(0,0);
//        KFs[0].changeMeasure(centre.x, centre.y);
//        KFs[0].rect = roi;
//        KFs[0].pos = centre;
////        KFs.push_back( obj);
//    } else {
//        double max_area = -1;
//        int idx=0;
//        cout << "KFs Size "<< KFs.size() << endl;
//        for(uint i = 0; i < KFs.size(); i++){
//            Rect rect = roi & KFs[i].rect;
//            double area = (roi & KFs[i].rect).area();
//            cout << area << " " << roi << " " << KFs[i].rect << endl;
//            if(max_area < area){
//                max_area = area;
//                idx = i;
//            }
//        }
//        cout <<  max_area << "  " << idx << endl;
//        if(max_area == -1){
//            cout << "nouveau object"<< endl;
//        } else {
//            KFs[idx].old_pos = KFs[idx].pos;
//            KFs[idx].pos = centre;
//            KFs[idx].rect = roi;
//            KFs[idx].changeMeasure(centre.x,centre.y);
//        }
//    }
//}

//int track_object(vector<detectedObject> &KFs, vector<Rect> &roi, vector<Point2f> &centre){

//    double max_area = -1;
//    int idx;

//    if(KFs.size() == 0){
//        for(uint i = 0; i < centre.size())
//        detectedObject obj();
//        obj.
//        KFs.push_back( detectedObject());
//    }

//    for(uint i = 0; i < KFs.size(); i++){

//        for(uint j = 0; j < roi.size(); j++ ){
//            double area = (roi[j] & KFs[i].rect).area();
//            if(max_area < area){
//                max_area = area;
//                idx = j;
//            }

//        }

//        if(max_area = -1){
//            cout << "nouveau object"<< endl;
//        } else {
//            KFs[i].old_pos = KFs[i].pos;
//            KFs[i].pos = centre[idx];
//            KFs[i].rect = roi[idx];
//        }

//    }
//}


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
    cvNamedWindow("perspective",1);

    for(uint i = 0; i <  350; i++)
        cap >> frame;

    Point2f p_src[4];
    Point2f p_dst[4];
    Mat M = computePerspectiveTransform(p_src,p_dst);

    double w = 30;
    for(int ind = 0;;ind++)
    {
        // Get image
        cap >> frame;
        if( frame.empty() )
            break;

        frame.copyTo(image);

        Mat pers = Mat(15*w,15*w,image.type());
        warpPerspective(image,pers,M,Size(15*w,15*w),INTER_NEAREST);
        flip(pers,pers,0);


        // Extract background/foregroung info
        bg(pers,fore);
        bg.getBackgroundImage(back);


        // Extract shape
        vector<Rect> roi = detector.update(pers, 30);
        vector<Point2f> centre;
        vector<Rect> interret = extractSubRegion(fore,roi,centre);


        for(uint i = 0; i < roi.size(); i++){
            if(roi[i].width+roi[i].height > 500)
                continue;
            track_object(KFs,roi[i],centre[i]);
            // drawing part
            rectangle(pers,roi[i],cvScalar(255,0,0));
            rectangle(pers,interret[i],cvScalar(0,255,0));
            circle(pers,centre[i],3,cvScalar(255,255,0),2);
            char buf[256];
            sprintf(buf,"(%3.0f,%3.0f)",centre[i].x, centre[i].y);
            putText(pers, buf, centre[i], FONT_HERSHEY_SCRIPT_SIMPLEX, 0.8, cvScalar(255,0,0));
        }

        imshow("perspective",pers);
        imshow("Motion", detector.getMotion() );
        imshow( "Image", image );

//        cvWaitKey(0);
        if( cvWaitKey(10) >= 0 )
            break;

        image.copyTo(image_prev);

    }
    cap.release();

    cvDestroyWindow( "Motion" );
    cvDestroyWindow( "Image" );



    return 0;
}

//#include <stdio.h>
//#include <errno.h>
//#include <signal.h>
//#include <string.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <netdb.h>
//#include <netinet/in.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>

//#include <iostream>

//#define SERVEURNAME "172.20.27.69" // adresse IP de mon serveur

//int to_server_socket = -1;

//int main ( int argc, char **argv )
//{

//    char *server_name = SERVEURNAME;
//    struct sockaddr_in serverSockAddr;
//    struct hostent *serverHostEnt;
//    long hostAddr;
//    long status;
//    char buffer[512];

//    bzero(&serverSockAddr,sizeof(serverSockAddr));
//    hostAddr = inet_addr(SERVEURNAME);
//    if ( (long)hostAddr != (long)-1)
//        bcopy(&hostAddr,&serverSockAddr.sin_addr,sizeof(hostAddr));
//    else
//    {
//        serverHostEnt = gethostbyname(SERVEURNAME);
//        if (serverHostEnt == NULL)
//        {
//            printf("gethost rate\n");
//            exit(0);
//        }
//        bcopy(serverHostEnt->h_addr,&serverSockAddr.sin_addr,serverHostEnt->h_length);
//    }
//    serverSockAddr.sin_port = htons(4200);
//    serverSockAddr.sin_family = AF_INET;

//    /* creation de la socket */
//    if ( (to_server_socket = socket(AF_INET,SOCK_STREAM,0)) < 0)
//    {
//        printf("creation socket client ratee\n");
//        exit(0);
//    }
//    /* requete de connexion */
//    if(connect( to_server_socket,
//                (struct sockaddr *)&serverSockAddr,
//                sizeof(serverSockAddr)) < 0 )
//    {
//        printf("demande de connection ratee\n");
//        exit(0);
//    }
//    /* envoie de donne et reception */
//    std::cout << "write to socket " << std::endl;
//    int test = 42;
//    char buff[256];
//    sprintf(buff,"%d",test);
//    write(to_server_socket,&buff,strlen(buff));
//    write(to_server_socket,&buff,strlen(buff));
//    write(to_server_socket,&buff,strlen(buff));
////    write(to_server_socket,&test,4);
//    read(to_server_socket,buffer,512);
////    printf(buffer);
//    /* fermeture de la connection */
//    shutdown(to_server_socket,2);
//    close(to_server_socket);
//}


#ifdef _EiC
main(1,"motempl.c");
#endif
