//
//  classifier.hpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 4/9/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//
//  This class implements a cascade classifier for calculating
//  score for each object detected by the tracking system.
//

#ifndef classifier_hpp
#define classifier_hpp

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/dpm.hpp>

#include <iostream>
#include <vector>

#include <stdio.h>

#include "dpm/dpm_cascade.hpp"

using namespace cv;
using namespace std;
using namespace cv::dpm;

class Classifier {
    // Video frame
    Mat frame;
    
    // Pedestrian cascade classifier
    cv::Ptr<DPMDetector> pedestrianClassifier;
    
    // Bicycles cascade classifier
    cv::Ptr<DPMDetector> bicycleClassifier;
    
    // Car cascade classifier
    cv::Ptr<DPMDetector> carClassifier;
    
    // Vector with objects detected.
    vector<Rect> objects;
    
    // Samples image counter
    int n_images;
    
public:
    Classifier();
    bool isBike(Mat, Rect object);
    bool isCar(Mat, Rect object);
    bool isPedestrian(Mat, Rect object);
    
};

#endif /* classifier_hpp */
