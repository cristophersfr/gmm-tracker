//
//  blob_detector.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 2/18/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include "blob_detector.hpp"

//Instant the Background Subtractor and its parameters.
BlobDetector::BlobDetector(){
    bgsubtractor = createBackgroundSubtractorMOG2();
    bgsubtractor->setNMixtures(3);
    bgsubtractor->setDetectShadows(false);
    bgsubtractor->setShadowValue(127);
}

//Return the foreground mask.
Mat BlobDetector::getFore(Mat frame){
    
    bgsubtractor->apply(frame, fore);
    
    return fore;
}