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
private:
    int id;
    // Video frame
    Mat frame;
    // Object bounding box
    Rect object;
    // HOG Descriptor
    HOGDescriptor pedestrian;
    // DPM Bicycles cascade classifier
    cv::Ptr<DPMDetector> bicycleClassifier;
    // Vector with objects detected.
    vector<Rect> detectedObjects;
    // Samples image counter
    int hits;
    // Counting how many frames used.
    int n_frames;
    // Total amount of score
    float total_ratio;
    bool isClassified;
    bool result;
    
    Rect createROI(Rect object);
    Rect adjustBox(Rect objectROI, Mat frameROI);
    Rect readjustBox(Rect object, Rect objectROI);
    
public:
    Classifier();
    void update(Mat frame, Rect object);
    bool detectBicycles();
    bool detectPedestrians();
    Rect getObject();
    int getId();

};

#endif /* classifier_hpp */
