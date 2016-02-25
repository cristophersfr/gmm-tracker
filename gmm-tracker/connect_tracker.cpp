//
//  connect_tracker.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 2/21/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include "connect_tracker.hpp"

void connectTracker::init(vector<Rect> objectsWindows, Mat frame){
    vector<Rect> :: const_iterator itr = objectsWindows.begin();
    while(itr!=objectsWindows.end()){
        KCFTracker * tracker;
        tracker->init(*itr, frame);
        trackers.push_back(*tracker);
    }
}
