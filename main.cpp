//Class headers
#include "detection.hpp"
//Namespace decl.
using namespace cv;

int main(int, const char **) {
    {
        // Declare the output variables
        Detector d;
        Mat frame;
        auto video = false;

        //-------Static Image-------//
        if (video == false){
            // Loads an image
            frame = imread( "images/20.jpg", IMREAD_COLOR );
            // Check if image is loaded fine
            if(frame.empty()){
                printf(" Error opening image\n");
                return -1;
            }
            //Run the detector (detect houghlines)
            vector<Point2d> turbinePoints;
            turbinePoints = d.detect(frame);
            //d.locate(frame);
            cv::imshow("Output", frame);
        }


        //-------Video Feed-------//
        if (video == true){
            VideoCapture cap("videos/1.mp4"); // open the default camera
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
                //d.locate(frame);
                // Display image.
                cv::imshow("Output", frame);
                if (waitKey(5) >= 0)
                    break;
            }
        }
    // Declare the output variables
    Detector detect;
    // run the capture function.
    detect.capture();
    return 0;
}
