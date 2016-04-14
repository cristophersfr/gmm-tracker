//
//  classifier.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 4/9/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include "classifier.hpp"

Classifier::Classifier(){
    
//    pedestrianClassifier = \
//    DPMDetector::create(vector<string>(1, "/Users/cristopher/Workspace/ \
//                                       gmm-tracker/gmm-tracker/dpm-models/inriaperson.xml"));
    
    bicycleClassifier = \
    DPMDetector::create(vector<string>(1, "/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/dpm-models/bicycles.xml"));
    
//    carClassifier = \
//    DPMDetector::create(vector<string>(1, "/Users/cristopher/Workspace/ \
//                                       gmm-tracker/gmm-tracker/dpm-models/car.xml"));
}

bool Classifier::isBike(Mat frame, Rect object){
    // Vector with objects detects in the frame.
    vector<DPMDetector::ObjectDetection> ds;
    
    // Expand size of the rectangle to guarantee a better detection.
    object += object.size();
    object += object.size();
    object = object - Point(object.width/2,object.height/2);
    
    // Create an object mask from its frame.
    Mat frameROI = Mat(frame, object);
    
    // specify fx and fy and let the function compute the destination image size.
    resize(frameROI, frameROI, Size(), 3, 3);
    
    // Detect the object.
    bicycleClassifier->detect(frameROI, ds);
    
    // Verify its score.
    cout << ds.size();
    
    // Return true or false.
    return false;
}


