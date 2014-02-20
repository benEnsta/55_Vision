#include <iostream>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

Point2f computeIntersect(Vec2f line1, Vec2f line2);
vector<Point2f> lineToPointPair(Vec2f line);
bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta);

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold = 14;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;



int grid_detection(Mat img)
{
    /*Mat occludedSquare = imread("/home/monnetdo/Download/lala.png");

    resize(occludedSquare, occludedSquare, Size(0, 0), 0.25, 0.25);

    Mat occludedSquare8u;
    cvtColor(occludedSquare, occludedSquare8u, CV_BGR2GRAY);

    Mat thresh;
    threshold(occludedSquare8u, thresh, 200.0, 255.0, THRESH_BINARY);

    GaussianBlur(thresh, thresh, Size(7, 7), 2.0, 2.0);

    Mat edges;
    Canny(thresh, edges, lowThreshold, lowThreshold*ratio, kernel_size );*/

  src = imread("/home/biop/55_vision/binarisation/lala2.png");

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);

    vector<Vec2f> lines;
    HoughLines( detected_edges, lines, 1, CV_PI/180, 50, 0, 0 );

    //cout << "Detected " << lines.size() << " lines." << endl;

    // compute the intersection from the lines detected...
    vector<Point2f> intersections;
    vector<Point2f> keep;
    int t;
    int flag = 1;
    for( size_t i = 0; i < 70; i++ )//lines.size(); i++ )
    {
        for(size_t j = 0; j < 70; j++) //lines.size(); j++)
        {
            Vec2f line1 = lines[i];
            Vec2f line2 = lines[j];
            if(acceptLinePair(line1, line2, CV_PI / 32))
            {
                Point2f intersection = computeIntersect(line1, line2);
                intersections.push_back(intersection);
                t++;
            }
        }
        //cout << "there are " << t<< "intersections "  << endl;

    }

    if(intersections.size() > 0)
    {
        vector<Point2f>::iterator i;
        vector<Point2f>::iterator j;
        for(i = intersections.begin(); i != intersections.end(); ++i)
        {
            for(j = keep.begin(); j != keep.end(); ++j)
            {

                if ( (i->x-j->x)*(i->x-j->x)+(i->y-j->y)*(i->y-j->y) < 500)
                {
                    flag = 0;
                }

            }
            if(flag != 0)
            {
                Point2f intersection;

                keep.push_back(*i);
            }
            flag=1;
           // cout << "Intersection is " << i->x << ", " << i->y << endl;
            //circle(src, *i, 1, Scalar(0, 255, 0), 3);

        }
        for(i = keep.begin(); i != keep.end(); ++i)
        {
            circle(src, *i, 1, Scalar(0, 0, 255), 3);
        }
        //cout << keep.size() << " point keep" << endl;
    }
    Point2f ref = keep.at(1);
    vector<Point2f>::iterator k;
    vector<Point2f>::iterator p;
    vector<Point2f>::iterator z;
    Point2f point1,point2,point3;
    for(k = keep.begin(); k != keep.end(); ++k)
    {

        if (k->x-k->y < ref.x-ref.y && k->x > 0 && k->y > 0 && k->x < 480 && k->y < 680 )
        {
            ref = *k;
            //cout << "point ref" << ref.x << "   "<<ref.y << endl;
        }


    }


    for(p = keep.begin(); p != keep.end(); ++p)
    {

        if ((p->x-ref.x)*(p->x-ref.x)+(p->y-ref.y)*(p->y-ref.y)<
                (point1.x-ref.x)*(point1.x-ref.x)+(point1.y-ref.y)*(point1.y-ref.y) && *p != ref)
        {
            point1 = *p;
        }

    }
        for(p = keep.begin(); p != keep.end(); ++p)
        {
            if ((p->x-ref.x)*(p->x-ref.x)+(p->y-ref.y)*(p->y-ref.y)<
                    (point2.x-ref.x)*(point2.x-ref.x)+(point2.y-ref.y)*(point2.y-ref.y) && *p != ref && *p != point1
                    && p->x > 0 && p->x < 480 && p->y > 0 && p->y < 680 )
            {
                point2 = *p;
            }
        }
        for(p = keep.begin(); p != keep.end(); ++p)
        {
            if ((p->x-ref.x)*(p->x-ref.x)+(p->y-ref.y)*(p->y-ref.y)<
                    (point3.x-ref.x)*(point3.x-ref.x)+(point3.y-ref.y)*(point3.y-ref.y) && *p != ref && *p != point1
                    && *p != point2&& p->x > 0 && p->x < 480 && p->y > 0 && p->y < 680 )
            {
                point3 = *p;
            }
        }



    int cpt=0;
    int cpt2 = 0;
    cv::Point2f source_points[3];
    cv::Point2f dest_points[3];
    source_points[2] = ref;
    float b = (ref.y-point2.y)/(ref.x-point2.x);
    int a = ref.y - b*ref.x;
    float d = (ref.y-point3.y)/(ref.x-point3.x);
    int c = ref.y - d*ref.x;
    for(z = keep.begin(); z != keep.end(); ++z)
    {
        //cout << "res = " <<  a+b*z->x << " expected"<<ref.y << " ="<< a+b*ref.x<< endl;
        if (z->y > (a-10)+b*z->x && z->y < (a+10)+b*z->x )
        {
            cpt ++;
            circle(src, *z, 1, Scalar(255, 0, 0), 3);
            if ((z->x-ref.x)*(z->x-ref.x)+(z->y-ref.y)*(z->y-ref.y)>
                    (source_points[1].x-ref.x)*(source_points[1].x-ref.x)+(source_points[1].y-ref.y)*(source_points[1].y-ref.y)
                    && z->x > 0 && z->x < 480 && z->y > 0 && z->y < 680 )
            {

                source_points[1] = *z;
            }

            cout << source_points[1].x << "   "<< source_points[1].y << endl;

        }
        if (z->y > (c-10)+d*z->x && z->y < (c+10)+d*z->x )
        {
            cpt2 ++;
            circle(src, *z, 1, Scalar(255, 0, 0), 3);
            if ((z->x-ref.x)*(z->x-ref.x)+(z->y-ref.y)*(z->y-ref.y)>
                    (source_points[2].x-ref.x)*(source_points[2].x-ref.x)+(source_points[2].y-ref.y)*(source_points[2].y-ref.y))
            {

                source_points[2] = *z;
            }



        }
        circle(src, source_points[2], 1, Scalar(0, 25, 0), 3);
        circle(src, source_points[1], 1, Scalar(0, 25, 0), 3);
    }
    circle(src, ref, 1, Scalar(0, 255, 0), 3);

    source_points[0] = ref;

    dest_points[0].x = 5;
    dest_points[0].y = 5;
    dest_points[1].x = 5;
    dest_points[1].y = cpt;
    dest_points[2].x = cpt2;
    dest_points[2].y = 5;

    cv::Mat dst;
    cv::Mat _transform_matrix = cv::getPerspectiveTransform(source_points, dest_points);
//    cv::warpPerspective(src, src, _transform_matrix, src.size());




    imshow("intersect", src);
    waitKey();

    return 0;
}




bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta)
{
    float theta1 = line1[1], theta2 = line2[1];

    if(theta1 < minTheta)
    {
        theta1 += CV_PI; // dealing with 0 and 180 ambiguities...
    }

    if(theta2 < minTheta)
    {
        theta2 += CV_PI; // dealing with 0 and 180 ambiguities...
    }

    return abs(theta1 - theta2) > minTheta;
}


// the long nasty wikipedia line-intersection equation...bleh...
Point2f computeIntersect(Vec2f line1, Vec2f line2)
{
    vector<Point2f> p1 = lineToPointPair(line1);
    vector<Point2f> p2 = lineToPointPair(line2);

    float denom = (p1[0].x - p1[1].x)*(p2[0].y - p2[1].y) - (p1[0].y - p1[1].y)*(p2[0].x - p2[1].x);
    Point2f intersect(((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].x - p2[1].x) -
                       (p1[0].x - p1[1].x)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom,
                      ((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].y - p2[1].y) -
                       (p1[0].y - p1[1].y)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom);

    return intersect;
}

vector<Point2f> lineToPointPair(Vec2f line)
{
    vector<Point2f> points;

    float r = line[0], t = line[1];
    double cos_t = cos(t), sin_t = sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    points.push_back(Point2f(x0 + alpha*(-sin_t), y0 + alpha*cos_t));
    points.push_back(Point2f(x0 - alpha*(-sin_t), y0 - alpha*cos_t));

    return points;
}
