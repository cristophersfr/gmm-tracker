//
//  connectTracker.hpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 2/21/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#ifndef connectTracker_hpp
#define connectTracker_hpp

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdio.h>

#include "kcftracker/kcftracker.hpp"

using namespace std;
using namespace cv;

class connectTracker {
    vector<KCFTracker> trackers;
    bool isTracking;
    
public:
    connectTracker();
    void run(vector<Rect> objectsWindows);
};

#endif /* connectTracker_hpp */
