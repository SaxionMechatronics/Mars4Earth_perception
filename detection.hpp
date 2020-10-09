/**
  @file     detection.h
  @project  Turbine detection
  @author   Thijs de Jong
  @email    thijsdejong21@gmail.com
  @date     16th of May, 2019
*/

//Prevent over definition, remove this and your compiler will crash/freeze :o
//#pragma once

//Library heaaders
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>


//Namespace decl.
using namespace std; 
using namespace cv;
using namespace cv::xfeatures2d;

/*
  @Class: Detector

  @functions (public):
    Detector()
        Initializes the sliders and its windows

    void  showVideo(Mat frame)
        Debug function that displays the given "frame" in a window created within its scope

    void  locate(Mat frame)
        PnP Magic, WIP

    void  detect(Mat frame)
        Uses the video selector and the input frame to detect houghlines.
        input:
            frame: 
                CV::Mat format
        outputs:
            It spawns named windows.
    void  detectKeys(bool video, Mat frame)
*/

class Detector 
{  
    //=================== Initializer ===================
    public:
    Detector();

    //================ Private functions ===============
    private: 


    struct isEqual //used in lines2point, determines which lines are colliding 
    {
        bool operator()(const Vec4i& _l1, const Vec4i& _l2) 
        {
            Vec4i l1(_l1), l2(_l2);

            float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
            float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));

            float product = (l1[2] - l1[0])*(l2[2] - l2[0]) + (l1[3] - l1[1])*(l2[3] - l2[1]);

            if (fabs(product / (length1 * length2)) < cos(CV_PI / 30))
                return false;

            float mx1 = (l1[0] + l1[2]) * 0.5f;
            float mx2 = (l2[0] + l2[2]) * 0.5f;

            float my1 = (l1[1] + l1[3]) * 0.5f;
            float my2 = (l2[1] + l2[3]) * 0.5f;
            float dist = sqrtf((mx1 - mx2)*(mx1 - mx2) + (my1 - my2)*(my1 - my2));

            if (dist > std::max(length1, length2) * 0.5f)
                return false;

            return true;
        }
    };
        

    //================ Public variables ================
    public: 
    
    // Output rotation and translation
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;
    vector<Vec4i> lines;

    //dummy var
    string name;

    //================ Public functions ================
    public:

    //[80% translation stable, rotation is instable (flips positivity) ]
    //PnP magic
    void locate(Mat frame);

    //[30% - Uncalibrated - incomplete]
    //Turbine detector using houghlines
    vector<Point2d> detect(Mat frame);

    //[100% - Finished]
    //creates a rectangle around the given vectors
    Rect lines2boundingbox(Mat frame, vector<Vec4i> lines);

    //[100% - Finished]
    //Groups lines based on size and angle
    vector<Vec4i> lines2points( vector<Vec4i> lines );

    void blade(Mat frame, vector<Vec4i> lines);

};