#include <cstdio>
#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
//Class headers
#include "detection.hpp"

//Namespace decl.
using namespace cv;

int main(int argc, char** argv)
{
    // Declare the output variables
    Detector d;
    const char* default_file = "images/1.jpg";
    const char* filename = argc >=2 ? argv[1] : default_file;
    // Loads an image
    Mat src = imread( samples::findFile( filename ), IMREAD_COLOR );
    // Check if image is loaded fine
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", default_file);
        return -1;
    }

    //Run the detector (detect houghlinees)
    imshow("Input", src);
    vector<Point2d> turbinePoints;
    turbinePoints = d.detect(src);
    d.locate(src);

    // Wait and Exit
    waitKey();
    return 0;
}
