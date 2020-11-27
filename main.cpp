#include <iostream>
#include <opencv2/imgcodecs.hpp>
//Class headers
#include "detection.hpp"
//Namespace decl.
using namespace cv;

int main(int, const char **) {
    // Declare the output variables
    Detector detect;
    Mat frame;
    detect.capture();
    return 0;
}
