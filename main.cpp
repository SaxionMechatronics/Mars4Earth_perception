#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
//Class headers
#include "detection.hpp"


//Namespace decl.
using namespace cv;

int main(int argc, const char** argv)
{
    // Declare the output variables
    Detector d;
    Mat frame;

    //-------Static Image-------//
//    const char* default_file = "images/7.png";
//    const char* filename = argc >=2 ? argv[1] : default_file;
//    // Loads an image
//    frame = imread( samples::findFile( filename ), IMREAD_COLOR );
//    // Check if image is loaded fine
//    if(frame.empty()){
//        printf(" Error opening image\n");
//        printf(" Program Arguments: [image_name -- default %s] \n", default_file);
//        return -1;
//    }
//    //Run the detector (detect houghlines)
//    vector<Point2d> turbinePoints;
//    turbinePoints = d.detect(frame);
//    d.locate(frame);

    //-------Video Feed-------//
    VideoCapture cap("videos/2.mp4"); // open the default camera
    if(!cap.isOpened()) { // check if we succeeded
        std::cout << "cannot open camera "<< std::endl;
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    std::cout << "Start grabbing" << std::endl
         << "Press any key to terminate" << std::endl;
    for (;;)
    {
        cap >> frame; // get a new frame from camera
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        //Run the detector (detect houghlines)
        vector<Point2d> turbinePoints;
        turbinePoints = d.detect(frame);
        d.locate(frame);
        if (waitKey(5) >= 0)
            break;
    }

    // Wait and Exit
    waitKey();
    return 0;
}
