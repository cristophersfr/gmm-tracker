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
    
    //Applying bg subtraction.
    bgsubtractor->apply(frame, fore);
    
    //Removing noise from foreground.
    medianBlur(fore, fore, 3);
    
    //Closing objects.
    int morph_size = 3;
    Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    morphologyEx(fore, fore, CV_MOP_CLOSE, element);
    
    //Removing shadows.
    threshold(fore, fore, 200, 255, THRESH_BINARY);
    
    return fore;
    
}

//Check if some window overlap.
void BlobDetector::checkWindowsOverlap(Rect r0){
    
    vector<Rect> :: const_iterator itr = trackedWindows.begin();
    vector<Point2f> points;
    
    while(itr!=trackedWindows.end()){
        Rect intersection = (r0 & *itr);
        if(intersection.area() > 100){
            points.push_back(Point2f(r0.x,r0.y));
            points.push_back(Point2f(r0.x + r0.width,r0.y + r0.height));
            points.push_back(Point2f(itr->x,itr->y));
            points.push_back(Point2f(itr->x + itr->width,itr->y + itr->height));
            Rect ret = boundingRect(points);
            points.clear();
            trackedWindows.erase(itr);
            trackedWindows.insert(itr,ret);
        }
        itr++;
    }
}

Mat BlobDetector::getBLOBS(){
    //Labeling components.
    Mat labelImage(fore.size(), CV_32S);
    int nLabels = connectedComponents(fore, labelImage, 8);
    std::vector<Vec3b> colors(nLabels);
    colors[0] = Vec3b(0, 0, 0);//background
    
//    for(int label = 1; label < nLabels; ++label){
//        colors[label] = Vec3b(rand()&255,rand()&255,rand()&255);
//    }
    
    //Getting labeled points.
    Mat dst(fore.size(), CV_8UC3);
    vector<Point3f> points;
    for(int r = 0; r < dst.rows; ++r){
        for(int c = 0; c < dst.cols; ++c){
            int label = labelImage.at<int>(r, c);
            points.push_back(Point3f(r,c,label));
//            Vec3b &pixel = dst.at<Vec3b>(r, c);
//            pixel = colors[label];
        }
    }
    
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
        if(r0.area() > 500){
            checkWindowsOverlap(r0);
            trackedWindows.push_back(r0);
        }
    }
    
    return dst;
    
}

Mat BlobDetector::drawTrackedWindows(Mat frame){
    vector<Rect> :: const_iterator itr = trackedWindows.begin();
    
    while(itr!=trackedWindows.end()){
        rectangle(frame, *itr, Scalar(255, 0, 0));
        Point2f center = Point2f((*itr).x + (*itr).width/2,  (*itr).y+ (*itr).height/2);
        circle(frame, center, 1, Scalar(255, 0, 0), 2, 8, 0);
        itr++;
    }
    
    return frame;
}

void BlobDetector::run(){
    
}