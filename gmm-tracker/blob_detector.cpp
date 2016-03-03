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
//    bgsubtractor->setHistory(120);
    bgsubtractor->setNMixtures(3);
    bgsubtractor->setDetectShadows(true);
    bgsubtractor->setShadowValue(127);
}

//Return the foreground mask.
Mat BlobDetector::getFore(Mat frame){
    
    this->frame = frame;
    
    //Applying bg subtraction.
    bgsubtractor->apply(frame, fore);
    
    //Removing noise from foreground.
    medianBlur(fore, fore, 3);
    
    //Closing objects.
    int morph_size = 7;
    Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    morphologyEx(fore, fore, CV_MOP_CLOSE, element);
    
    //Removing shadows.
    threshold(fore, fore, 200, 255, THRESH_BINARY);
    
    return fore;
    
}

//Check if box is moving.
bool BlobDetector::checkBoxMoving(vector<Rect> * windows, Rect r0){
    vector<Rect> :: const_iterator itr = windows->begin();
    vector<Point2f> points;
    
    while(itr!=windows->end()){
        Rect intersection = (r0 & *itr);
        if(intersection.area() > 0){
            //Remove the previous box.
            windows->erase(itr);
            //Insert the most recent box.
            windows->push_back(r0);
            return true;
        }
        itr++;
    }
    
    return false;
}

//Check if some window overlap.
void BlobDetector::checkWindowsOverlap(vector<Rect> * windows, Rect r0){
    
    vector<Rect> :: const_iterator itr = windows->begin();
    
    while(itr!=windows->end()){
        Rect intersection = (r0 & *itr);
        if(intersection.area() > 100){
            intersection += intersection.size();
            //windows->erase(itr);
            windows->insert(itr,intersection);
        }
        itr++;
    }
}

//Getting BLOBS.
Mat BlobDetector::getBLOBS(){
    
    trackedWindows = vector<Rect>();
    
    //Labeling components.
    Mat labelImage(fore.size(), CV_32S);
    int nLabels = connectedComponents(fore, labelImage, 8);
    std::vector<Vec3b> colors(nLabels);
    colors[0] = Vec3b(0, 0, 0);//background
    
    for(int label = 1; label < nLabels; ++label){
        colors[label] = Vec3b(rand()&255,rand()&255,rand()&255);
    }
    
    //Getting labeled points.
    Mat dst(fore.size(), CV_8UC3);
    vector<Point3f> points;
    for(int r = 0; r < dst.rows; ++r){
        for(int c = 0; c < dst.cols; ++c){
            int label = labelImage.at<int>(r, c);
            points.push_back(Point3f(r,c,label));
            Vec3b &pixel = dst.at<Vec3b>(r, c);
            pixel = colors[label];
        }
    }
    
    //cout << nLabels << endl;
    
    //Drawing bounding boxes around components.
    vector<Point3f> :: const_iterator itl = points.begin();
    vector<Point2f> rect_points;
    
    for(int label = 1; label < nLabels; ++label){
        while(itl!=points.end()){
            Point3f p = *itl;
            
            if(p.z == float(label)){
                rect_points.push_back(Point2f(p.y, p.x));
            }
            
            itl++;
        }
        itl = points.begin();
        Rect r0 = boundingRect(rect_points);
        rect_points.clear();
        if(r0.area() > 400 && r0.area() < 1000){
            checkWindowsOverlap(&trackedWindows, r0);
            trackedWindows.push_back(r0);
        }
    }
    
    return dst;
    
}

//Get moving object using ROI.
vector<Rect> BlobDetector::getMovingObjects(){
    vector<Rect> detectedWindows;
    
    Rect detectionROI = Rect(240,100,5,120);
    rectangle(frame, detectionROI, Scalar(0, 0, 255));
    
    vector<Rect> :: const_iterator itr = trackedWindows.begin();
    
    while(itr!=trackedWindows.end()){
        Rect intersection = (detectionROI & *itr);
        if(intersection.area() > 0){
            if(itr->area() > 400 && itr->area() < 1000){
                //bool result = checkBoxMoving(&detectedWindows, *itr);
                //if(!result){
                    detectedWindows.push_back(*itr);
                //}
            }
        }
        itr++;
    }
    
    
    return detectedWindows;
}


//Drawing tracked windows.
Mat BlobDetector::drawTrackedWindows(){
    vector<Rect> :: const_iterator itr = trackedWindows.begin();
    
    while(itr!=trackedWindows.end()){
        rectangle(frame, *itr, Scalar(255, 0, 0));
        Point2f center = Point2f((*itr).x + (*itr).width/2,  (*itr).y+ (*itr).height/2);
        circle(frame, center, 1, Scalar(255, 0, 0), 2, 8, 0);
        itr++;
    }
    
    return frame;
}

//Drawing detected windows.
Mat BlobDetector::drawDetectedWindows(vector<Rect> detectedWindows){
    vector<Rect> :: const_iterator itr = detectedWindows.begin();
    
    while(itr!=detectedWindows.end()){
        rectangle(frame, *itr, Scalar(0, 0, 255));
        Point2f center = Point2f((*itr).x + (*itr).width/2,  (*itr).y+ (*itr).height/2);
        circle(frame, center, 1, Scalar(0, 0, 255), 2, 8, 0);
        itr++;
    }
    
    return frame;
}


void BlobDetector::run(){
    
}