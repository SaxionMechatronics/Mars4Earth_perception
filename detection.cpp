/**
  @file     detection.h
  @project  Turbine detection
  @author   Thijs de Jong
  @email    thijsdejong21@gmail.com
  @date     16th of May, 2019
*/
//class header
#include "detection.hpp"
//Namespace
using namespace std; 
using namespace cv;
using namespace cv::xfeatures2d; 

Detector::Detector(){

    };

void Detector::locate(Mat frame) {
    // -------- gray scaling ----------
    Mat blob;
    // Convert from BGR to GRAY colorspace
    cvtColor(frame, blob, COLOR_BGR2GRAY);
    // -------- Blob detection ----------
    // Set up the detector with default parameters.
    // Detect blobs.
    std::vector<KeyPoint> keypoints;

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;
    
    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;
    
    // Filter by Area.
    params.filterByArea = false;
    params.minArea = 1000;
    
    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.8;
    
    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.87;
    
    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;

    //Run the blob detector
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect( blob, keypoints );
    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints;
    drawKeypoints( blob, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Show blobs
    imshow("keypoints", im_with_keypoints );

    //Debug print
    for( int i = 0 ; i < keypoints.size(); i++)
    {
        cout << "Blob " << i << " : " << keypoints[i].pt << "\t";
    }
    cout << "\n\n";
    //PnP magic ...
    std::vector<cv::Point2d> image_points;

    if(keypoints.size() == 5)
    {
        //Fill image_points using blob detection

        image_points.push_back( cv::Point2d(keypoints[0].pt.x, keypoints[0].pt.y) );    
        image_points.push_back( cv::Point2d(keypoints[1].pt.x, keypoints[1].pt.y) );    
        image_points.push_back( cv::Point2d(keypoints[2].pt.x, keypoints[2].pt.y) );    
        image_points.push_back( cv::Point2d(keypoints[3].pt.x, keypoints[3].pt.y) ); 
        image_points.push_back( cv::Point2d(keypoints[4].pt.x, keypoints[4].pt.y) ); 
    
        // 3D model points.
        std::vector<cv::Point3d> model_points;
                 
        model_points.push_back(cv::Point3d(  25.0f,25.0f, 0.0f));  //right top         
        model_points.push_back(cv::Point3d(-25.0f,-25.0f, 0.0f));  //left bottom    
        model_points.push_back(cv::Point3d(-25.0f, 25.0f, 0.0f));  //left top
        model_points.push_back(cv::Point3d( 0.0f,-25.0f, 0.0f));   //bottom
        model_points.push_back(cv::Point3d(  0.0f,  0.0f, 0.0f));  //Center  
        
        // Camera internals
        double focal_length = frame.cols; // Approximate focal length.
        Point2d center = cv::Point2d(frame.cols/2,frame.rows/2);
        cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
        cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
        
        cout << "Camera Matrix " << endl << camera_matrix << endl ;
        // Solve for pose
        cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
        
        // Project a axis onto the image plane.
        vector<Point3d> nose_end_point3D;
        vector<Point2d> nose_end_point2D;
        nose_end_point3D.push_back(Point3d(100,0,0));
        nose_end_point3D.push_back(Point3d(0,100,0));
        nose_end_point3D.push_back(Point3d(0,0,100));
        
        projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);

        for(auto & image_point : image_points)
        {
            circle(frame, image_point, 3, Scalar(0,0,255), -1);
        }
        
        cv::line(frame,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);
        cv::line(frame,image_points[0], nose_end_point2D[1], cv::Scalar(0,255,0), 2);
        cv::line(frame,image_points[0], nose_end_point2D[2], cv::Scalar(0,0,255), 2);

        cout << "Rotation Vector " << endl << rotation_vector << endl;
        cout << "Translation Vector" << endl << translation_vector << endl;
        
        cout <<  nose_end_point2D << endl;
    }
}

vector<Point2d> Detector::detect(Mat frame){
    // -------- gray scaling ----------
    Mat BW;
    cvtColor(frame, BW, COLOR_BGR2GRAY);
//    // -------- Skeleton ---------- //
//    cv::Mat skel(BW1.size(), CV_8UC1, cv::Scalar(0));
//    cv::Mat temp;
//    cv::Mat eroded;
//
//    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(5, 5));
//    bool done;
//    int iterations=0;
//    do
//    {
//        erode(BW1, eroded, element);
//        dilate(eroded, temp, element);
//        subtract(BW1, temp, temp);
//        bitwise_or(skel, temp, skel);
//        eroded.copyTo(BW1);
//
//        done = (countNonZero(BW1) == 0);
//        iterations++;
//
//    } while (!done && (iterations < 25));
//    //------- Dilate -------------//
//    Mat BW1_5;
//    dilate(skel,BW1_5,element);
//    imshow("bw1.5: skeleton", BW1_5);

    //-------Canny----------//
    Mat cannyT, gBlur, mBlur;
    GaussianBlur(BW,gBlur, Size(5, 5), 3 );
    medianBlur ( gBlur, mBlur, 3 );
    Canny(mBlur,cannyT,75, 200, 3, true);
    imshow("bw1.5: Canny", cannyT);
    //-------Hough lines----------//
    /*
    dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
    lines: A vector that will store the parameters (x_{start}, y_{start}, x_{end}, y_{end}) of the detected lines
    rho : The resolution of the parameter r in pixels. We use 1 pixel.
    theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
    threshold: The minimum number of intersections to “detect” a line
    minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    maxLineGap: The maximum gap between two points to be considered in the same line.  */
    vector<Vec4i> lines;
    HoughLinesP( cannyT, lines, 1, CV_PI/360, 50, 50, 10 );
    for(auto & i : lines)
    {
        line(frame, Point(i[0], i[1]), Point(i[2], i[3]), Scalar(0, 0, 255), 2, LINE_AA );
    }
    Rect boundingBox = lines2boundingbox(frame, lines);

    //cout << "HLT1 \t box: " << 1 << "\t Xmin, Xmax: (" << Xmin << "," << Xmax << ") \t Ymin,Ymax: (" << Ymin << "," << Ymax  << ")\n";
    rectangle(frame, boundingBox, Scalar(0, 0, 0,255));
    //imshow("bw4: Hough lines", HLT1);
    //Mat& img, Rect rec, const Scalar& color,

    //Group overlaying lines together
    vector<Vec4i> corelines = lines2points(lines);

    //----- Fabricate points for the locate alg. -----------//
    Size s = frame.size();
    // rows = s.height;
    // cols = s.width;
    vector<Point2d> output;
    vector<Point2d> centerpoints;
    Mat circleEnd = frame;
    //circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)

    //Determinens which end points of the found core lines are in the middle of the screen and which one are on the edges
    for(auto & coreline : corelines)
    {
        if( ( coreline[0] < (s.width/2 - s.width/4) || coreline[0] > (s.width/2 + s.width/4) ) ||
            ( coreline[1] < (s.height/2 - s.height/4) || coreline[1] > (s.height/2 + s.height/4) )  )
        {
            //circle(circle,Point2d(corelines[i][0], corelines[i][1]), 5, Scalar(0,0,255),2); //red
            output.emplace_back(coreline[0], coreline[1]);
        }
        else
        {
            //circle(circle,Point2d(corelines[i][0], corelines[i][1]), 5, Scalar(255,255,0),2); //blue
            centerpoints.emplace_back(coreline[0], coreline[1]);
        }

        if( ( coreline[2] < (s.width/2 - s.width/4) || coreline[2] > (s.width/2 + s.width/4) ) ||
            (coreline[3] < (s.height/2 - s.height/4) || coreline[3] > (s.height/2 + s.height/4)))
        {
            //circle(circle,Point2d(corelines[i][2], corelines[i][3]), 6, Scalar(0,0,255),2);
            output.emplace_back(coreline[2], coreline[3]);
        }
        else
        {
            //circle(circle,Point2d(corelines[i][2], corelines[i][3]), 6, Scalar(255,255,0),2);
            centerpoints.emplace_back(coreline[2], coreline[3]);
        }
        line( circleEnd, Point(coreline[0], coreline[1]),
            Point(coreline[2], coreline[3]), Scalar(0,255,0), 2, 8 );
    }

    //resolve middle point of the wind turbine
    int x_acc = 0;
    int y_acc = 0;
    for(auto & centerpoint : centerpoints)
    {
        x_acc += centerpoint.x;
        y_acc += centerpoint.y;
    }

    if(!centerpoints.empty()){
        output.emplace_back(x_acc / centerpoints.size(), y_acc / centerpoints.size());
    }
    //debug print
    for(size_t i = 0; i < output.size(); i++ ){
        circle(circleEnd,Point2d(output[i].x, output[i].y), 6, Scalar(255,0,0),2);
    }
    return output;
}

Rect Detector::lines2boundingbox(Mat frame, vector<Vec4i> lines){
    int Xmax = frame.size[0]/2;
    int Xmin = frame.size[0]/2;
    int Ymax = frame.size[1]/2;
    int Ymin = frame.size[1]/2;

    Rect boundingBox;
    for( size_t i = 0; i < lines.size(); i++ ){
        //Create bounding box
        if (Xmax < lines[i][0] || Xmax < lines[i][2] ){
            if( lines[i][0] > lines[i][2]){
                Xmax = lines[i][0];
            }
            else{
                Xmax = lines[i][2];
            }
        }
        if (Xmin > lines[i][0] || Xmin > lines[i][2] ){
            if( lines[i][0] < lines[i][2]){
                Xmin = lines[i][0];
            }
            else {
                Xmin = lines[i][2];
            }
        }
        if (Ymax < lines[i][1] || Ymax < lines[i][3] ){
            if( lines[i][1] > lines[i][3]){
                Ymax = lines[i][1];
            }
            else{
                Ymax = lines[i][3];
            }
        }
        if (Ymin > lines[i][1] || Ymin > lines[i][3] ){
            if( lines[i][1] < lines[i][3]){
                Ymin = lines[i][1];
            }
            else{
                Ymin = lines[i][3];
            }
        }
    }
    boundingBox.x       = Xmin;
    boundingBox.y       = Ymin;
    boundingBox.width   = Xmax - Xmin; 
    boundingBox.height  = Ymax - Ymin;

    return boundingBox;
}

vector<Vec4i> Detector::lines2points( vector<Vec4i> lines ){
    //Catogorize lines based on angle :o 
    std::vector<int> labels;
    int numberOfLines = cv::partition(lines, labels, isEqual());

    //Output container
    vector<Vec4i> lines2;

    //For all labels
    for(int i = 0; i < numberOfLines; i++){
        vector<int> x;
        vector<int> y;
        bool big_x,big_y;

        //Check each line
        for(size_t j = 0; j < lines.size(); j++){
            //if line has the correct label
            if(labels[j] == i ){
                //Save values in the containers
                x.push_back( lines[i][0]);
                x.push_back( lines[i][2]);
                y.push_back( lines[i][1]);
                y.push_back( lines[i][3]);
            }
            
            //Save whether x1 is bigger than x2. Could also be done with the slope
            if(lines[i][0] < lines[i][2]){
                big_x = 1;
            }
            else{
                big_x = 0;
            }

            if(lines[i][1] < lines[i][3]){
                big_y = 1;
            }
            else{
                big_y = 0;
            }

        }

        //Determine the max and minima of the array
        auto x_min_value = *std::min_element(x.begin(),x.end());
        auto x_max_value = *std::max_element(x.begin(),x.end());
        auto y_min_value = *std::min_element(y.begin(),y.end());
        auto y_max_value = *std::max_element(y.begin(),y.end());
        
        //Create output containers
        int x1,x2,y1,y2;

        //Write maxima to the output containers according the arrangement determined before
        if(big_x){
            x1 = x_min_value;
            x2 = x_max_value;
        }
        else{
            x2 = x_min_value;
            x1 = x_max_value;
        }
        
        if(big_y){
            y1 = y_min_value;
            y2 = y_max_value;
        }
        else{
            y2 = y_min_value;
            y1 = y_max_value;
        }
        //Add the found line the 
        lines2.push_back( Vec4i(x1, y1, x2 , y2 ) );
    }
    return lines2;
}
