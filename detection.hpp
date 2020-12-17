/**
  @file     detection.h
  @project  Wind Turbine detection
  @author   Wytse de Witte
  @email    wytsedewitte@gmail.com
  @date     05-11-2020
*/

//Library heaaders
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

//Namespace decl.
using namespace std; 
using namespace cv;
using namespace cv::xfeatures2d;

class Detector 
{  
    //=================== Initializer ===================
    public:
    Detector();

    //================ Public variables ================
    public:
    VideoCapture cap;
    int apertureSize;
    int houghThreshold;
    int houghMinLength;
    int houghMaxLineGap;
    int fps = 0;
    vector<Vec4i> lines;
    vector<Vec4i> prev_lines;
    std::ofstream outputData;

    //================ Public functions ================
    public:
    // capture the image from camera or video
    int capture();
    //Turbine detector using houghlines
    void detect(Mat frame);
    //creates a rectangle around the given vectors
    Rect lines2boundingbox(Mat frame, vector<Vec4i> lines);
    //detects the blade of the wind turbine
    void blade(Mat frame, vector<Vec4i> lines);
    // Create a line memory (WIP, does not work)
    void lineMemory(Mat frame, vector<Vec4i> lines);
};