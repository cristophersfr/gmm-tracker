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
    // Detected object which can be used to recover the path.
    Rect lastDetected;
    // HOG Descriptor
    HOGDescriptor pedestrian;
    // DPM Bicycles cascade classifier
    cv::Ptr<DPMDetector> bicycleClassifier;
    // Samples image counter
    int bicycle_hits;
    int pedestrian_hits;
    // Counting how many frames used.
    int n_frames;

    bool isClassified;
    
    Rect createROI(Rect object);
    Rect adjustBox(Rect objectROI, Mat frameROI);
    Rect readjustBox(Rect object, Rect objectROI);
    
public:
    Classifier();
    void update(Mat frame, Rect object);
    int detectBicycles();
    int detectPedestrians();
    int getId();
    Rect getObject();

};

#endif /* classifier_hpp */
