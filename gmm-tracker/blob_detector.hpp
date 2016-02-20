//
//  blob_detector.hpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 2/18/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//
//  BlobDetector implements a background subtractor algorithm.
//

#ifndef blob_detector_hpp
#define blob_detector_hpp

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

class BlobDetector {

private:
    Mat fore, back;
    Ptr<BackgroundSubtractorMOG2> bgsubtractor;
    vector<Rect> trackedWindows;
    void checkBoxMoving();
    void checkWindowsOverlap(Rect r0);
    
public:
    BlobDetector();
    Mat getFore(Mat frame);
    Mat getBLOBS();
    Mat drawTrackedWindows(Mat frame);
    void run();
};

#endif /* blob_detector_hpp */
