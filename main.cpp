#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
//Class headers
#include "detection.hpp"
//Namespace decl.
using namespace cv;

int main(int, const char **) {
    // Declare the output variables
    Detector d;
    Mat frame;

    //-------Video Feed-------//
    VideoCapture cap; // open the default camera
    std::cout << "start" << std::endl;
    // open the default camera using default API
    cap.open(0);
    // OR advance usage: select any API backend
//    int deviceID = 1;             // 0 = open default camera
//    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
//    // open selected camera using selected API
//    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) { // check if we succeeded
        std::cout << "cannot open camera " << std::endl;
        return -1;
    }
    for (;;) {
        cap >> frame; // get a new frame from camera
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        //Run the detector (detect houghlines)
        d.detect(frame);
        // Display image.
        cv::imshow("Output", frame);
        if (waitKey(5) >= 0)
            break;
    }
    // Wait and Exit
    waitKey(0);
    return 0;
}
