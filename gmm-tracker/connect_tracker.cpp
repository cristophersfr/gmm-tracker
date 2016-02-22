//
//  connect_tracker.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 2/21/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include "connect_tracker.hpp"

void connectTracker::run(vector<Rect> objectsWindows){
    vector<Rect> :: const_iterator itr = objectsWindows.begin();
    while(itr!=objectsWindows.end()){
        KCFTracker kcft;
        //kcft.init(<#const cv::Rect &roi#>, <#cv::Mat image#>)
    }
}