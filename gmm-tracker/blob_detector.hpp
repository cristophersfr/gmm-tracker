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
#include <vector>
#include <map>

#include <stdio.h>

using namespace std;
using namespace cv;

class BlobDetector {

private:
    Mat frame;
    Mat fore, back;
    Ptr<BackgroundSubtractorMOG2> bgsubtractor;
    vector<Rect> trackedWindows;
    bool checkBoxMoving(vector<Rect> * windows, Rect r0);
    void checkWindowsOverlap(vector<Rect> * windows, Rect r0);
    
public:
    BlobDetector();
    Mat getFore(Mat frame);
    [[deprecated("Replaced by detectBLOBS, which has an improved implementation.")]]
    Mat getBLOBS();
    Mat detectBLOBS();
    vector<Rect> getMovingObjects();
    Mat drawTrackedWindows();
    Mat drawDetectedWindows(vector<Rect> detectedWindows);
    void run();
};

#endif /* blob_detector_hpp */
