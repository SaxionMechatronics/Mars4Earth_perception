/**
  @file     detection.h
  @project  Wind Turbine detection
  @author   Wytse de Witte
  @email    wytsedewitte@gmail.com
  @date     05-11-2020
*/
//class header
#include "detection.hpp"
//Namespace
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

Detector::Detector() = default;

void Detector::detect(Mat frame) {
    //-------Masking with HSV----------//
    Mat HSV, mask, HSVmasked, HSVoutput;
    // Convert from BGR to HSV colorspace
    cv::cvtColor(frame,HSV, COLOR_BGR2HSV);
    // Create a mask (this one takes all the brown and green)
    int low_H = 0, low_S = 20, low_V = 20;
    int high_H = 90, high_S = 255, high_V = 255;
    // Detect the object based on HSV Range Values
    inRange(HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), mask);
    // segment out the background
    bitwise_and(frame, frame, HSVmasked, mask = mask);
    cv::imshow("HSV_filter", HSVmasked);
    cv::absdiff(frame, HSVmasked, HSVoutput);
    //-------Canny----------//
    Mat cannyT, gBlur, mBlur;
    GaussianBlur(HSVoutput, gBlur, Size(5, 5), 5);
    medianBlur(gBlur, mBlur, 7);
    Canny(mBlur, cannyT, 50, 100, 3, true);
    //imshow("Canny", cannyT);
    //-------Hough lines----------//
    /*
    dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
    lines: A vector that will store the parameters (x_{start}, y_{start}, x_{end}, y_{end}) of the detected lines
    rho : The resolution of the parameter r in pixels. We use 1 pixel.
    theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
    threshold: The minimum number of intersections to “detect” a line
    minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    maxLineGap: The maximum gap between two points to be considered in the same line.  */
    HoughLinesP(cannyT, lines, 1, CV_PI / 360, 40, 100, 10);
    for (auto &i : lines) {
        line(frame, Point(i[0], i[1]), Point(i[2], i[3]), Scalar(0, 0, 255), 2, LINE_AA);
    }
    blade(frame, lines);
    Rect boundingBox = lines2boundingbox(frame, lines);
    rectangle(frame, boundingBox, Scalar(0, 0, 0, 255));
}

void Detector::blade(Mat frame, vector<Vec4i> lines) {
    vector<int> angle;
    float x1, y1, x2, y2;
    for (auto l : lines) {
        x1 = l[0];
        x2 = l[2];
        y1 = l[1];
        y2 = l[3];

        Point p1, p2;
        p1 = Point(x1, y1);
        p2 = Point(x2, y2);
        //calculate angle in radian, if you need it in degrees just do angle * 180 / PI
        angle.push_back(atan2(p1.y - p2.y, p1.x - p2.x) * 360 / CV_PI);
        for (int & i : angle) {
            if (i < 0) {
                i = atan2(p2.y - p1.y, p2.x - p1.x) * 360 / CV_PI;
            }
        }
        for (size_t i = 0; i < angle.size(); i++) {
            int temp;
            temp = angle[i];
            for (int j : angle) {
                int x, y;
                x = temp + j;
                y = temp - j;
                if (x > 110 && x < 150 || y > 110 && y < 150 ||
                        x > 210 && x < 260 || y > 210 && y < 260) {
                    //cout << "is blade: " << x << " : " << y << " : " << temp << endl;
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
