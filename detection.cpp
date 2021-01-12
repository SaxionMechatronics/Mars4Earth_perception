/**
  @file     detection.h
  @project  Wind Turbine detection
  @author   Wytse de Witte
  @email    wytsedewitte@gmail.com
  @date     05-11-2020
*/
//class header
#include <fstream>
#include "detection.hpp"
//Namespace
using namespace cv;
using namespace cv::xfeatures2d;

Detector::Detector() {
    apertureSize = 3;
    houghThreshold = 50;
    houghMinLength = 100;
    houghMaxLineGap = 15;
}

int Detector::capture() {
    // Declare the output variables
    Mat frame;
    /*
    // Loads an image //
    frame = imread( "images/20.jpg", IMREAD_COLOR );
    // Check if image is loaded fine
    if(frame.empty()){
        printf(" Error opening image\n");
        return -1;
    }*/

    /*
   // manual video //
   VideoCapture cap("videos/1.mp4"); // open the default camera
   if(!cap.isOpened()) { // check if we succeeded
       std::cout << "cannot open camera "<< std::endl;
       return -1;
   }

   -- GRAB AND WRITE LOOP
   std::cout << "Start grabbing" << std::endl
             << "Press any key to terminate" << std::endl;
   */

    //-------Video Feed-------//
    // open the default camera using default API
    cap.open(0);
    /// if you want to display videos
    // VideoCapture cap("videos/mini3.mp4");
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
        //cap >> frame; // get a new frame from camera
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        fps = cap.get(CAP_PROP_POS_FRAMES); // retrieves the current frame number
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        //Run the detector (detect houghlines)
        detect(frame);
        // Display image.
        cv::imshow("Output", frame);
        if (waitKey(5) >= 0)
            break;
    }
    // Wait and Exit
    waitKey(0);
    return 0;
}

void Detector::detect(Mat frame) {
    //-------Masking with HSV----------//
    Mat HSV, mask, HSVmasked;
    // Convert from BGR to HSV colorspace
    cv::cvtColor(frame, HSV, COLOR_BGR2HSV);
    // Create a mask (this one takes all the brown and green)
    int low_H = 85, low_S = 50, low_V = 50;
    int high_H = 120, high_S = 255, high_V = 255;
    // Detect the object based on HSV Range Values
    inRange(HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), mask);
    // segment out the background
    bitwise_or(frame, frame, HSVmasked, mask = mask);
    cv::imshow("HSV_filter", HSVmasked);
    //-------Canny----------//
    Mat cannyT, gBlur, mBlur;
    GaussianBlur(HSVmasked, gBlur, Size(5, 5), 5);
    medianBlur(gBlur, mBlur, 7);
    imshow("Gaussian & median blur", mBlur);
    Canny(mBlur, cannyT, 50, 100, apertureSize, true);
    imshow("Canny", cannyT);
    //-------Hough lines----------//
    /*
    dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
    lines: A vector that will store the parameters (x_{start}, y_{start}, x_{end}, y_{end}) of the detected lines
    rho : The resolution of the parameter r in pixels. We use 1 pixel.
    theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
    threshold: The minimum number of intersections to “detect” a line
    minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    maxLineGap: The maximum gap between two points to be considered in the same line.  */
    HoughLinesP(cannyT, lines, 1, CV_PI / 360, houghThreshold, houghMinLength, houghMaxLineGap);
    for (auto &i : lines) {
        line(frame, Point(i[0], i[1]), Point(i[2], i[3]), Scalar(0, 0, 255), 2, LINE_AA);
    }
    blade(frame, lines);
    Rect boundingBox = lines2boundingbox(frame, lines);
    rectangle(frame, boundingBox, Scalar(0, 0, 0, 255));
}

void Detector::blade(Mat frame, vector<Vec4i> lines) {
    vector<int> angle;
    int counter = 0;
    angle.push_back(5);

    float x1, y1, x2, y2;
    for (auto l : lines) {
        x1 = l[0];
        x2 = l[2];
        y1 = l[1];
        y2 = l[3];

        Point p1, p2;
        p1 = Point(x1, y1);
        p2 = Point(x2, y2);
        //counter += 1;
        //calculate angle in radian, if you need it in degrees just do angle * 180 / PI
        angle.at(counter) = atan2(p1.y - p2.y, p1.x - p2.x) * 360 / CV_PI;
        //check if angle isn't negative, if so change x,y points
        for (int i : angle) {
            if (i < 0) {
                int holdy;
                int holdx;
                holdy = p1.y;
                p1.y = p2.y;
                p2.y = holdy;
                holdx = p1.x;
                p1.x = p2.x;
                p2.x = holdx;
                angle.at(counter) = atan2(p1.y - p2.y, p1.x - p2.x) * 360 / CV_PI;
            }
        }

        outputData.open("output.txt", std::ios::app);
        outputData << p1.x << "," << p1.y << "," << p2.x << "," << p2.y << "\n";
        outputData.close();
        // for testing purposes.
//        if(fps <= 30){
//            std::cout << p1.x << "," << p1.y << "," << p2.x << "," << p2.y << std::endl;
//        }
        for (size_t i = 0; i < angle.size(); i++) {
            int startingAngle;
            startingAngle = angle[i];

            for (int currentAngle : angle) {
                int upperLimit, lowerLimit;
                upperLimit = startingAngle + currentAngle;
                lowerLimit = startingAngle - currentAngle;
                if (upperLimit > 110 && upperLimit < 130 || lowerLimit > 110 && lowerLimit < 130) {
                    //std::cout << "Blade starts at: x: " << p1.x << " y: " << p1.y << std::endl;
                    line(frame, Point(x1, y1), Point(x2, y2), Scalar(255, 0, 0), 1, LINE_AA);
                }
            }
        }
    }

}

Rect Detector::lines2boundingbox(Mat frame, vector<Vec4i> lines) {
    int Xmax = frame.size[0] / 2;
    int Xmin = frame.size[0] / 2;
    int Ymax = frame.size[1] / 2;
    int Ymin = frame.size[1] / 2;

    Rect boundingBox;
    for (auto &line : lines) {
        //Create bounding box
        if (Xmax < line[0] || Xmax < line[2]) {
            if (line[0] > line[2]) {
                Xmax = line[0];
            } else {
                Xmax = line[2];
            }
        }
        if (Xmin > line[0] || Xmin > line[2]) {
            if (line[0] < line[2]) {
                Xmin = line[0];
            } else {
                Xmin = line[2];
            }
        }
        if (Ymax < line[1] || Ymax < line[3]) {
            if (line[1] > line[3]) {
                Ymax = line[1];
            } else {
                Ymax = line[3];
            }
        }
        if (Ymin > line[1] || Ymin > line[3]) {
            if (line[1] < line[3]) {
                Ymin = line[1];
            } else {
                Ymin = line[3];
            }
        }
    }
    boundingBox.x = Xmin;
    boundingBox.y = Ymin;
    boundingBox.width = Xmax - Xmin;
    boundingBox.height = Ymax - Ymin;
    return boundingBox;
}

