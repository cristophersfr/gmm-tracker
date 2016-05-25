//
//  blob_detector.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 2/18/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include "blob_detector.hpp"

// Instantiate the Background Subtractor and its parameters.
BlobDetector::BlobDetector(int history, int nMixtures, bool detectShadows){
    bgsubtractor = createBackgroundSubtractorMOG2();
    bgsubtractor->setHistory(history);
    bgsubtractor->setNMixtures(nMixtures);
    bgsubtractor->setDetectShadows(detectShadows);
    bgsubtractor->setShadowValue(127);
}

// Return the foreground mask.
Mat BlobDetector::getFore(Mat frame){
    
    frame.copyTo(this->frame);
//    this->frame = frame;
    
    //Applying bg subtraction.
    bgsubtractor->apply(this->frame, fore);
    
    //Removing noise from foreground.
    medianBlur(fore, fore, 3);

    //Closing objects.
    int morph_size = 5;
    Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    morphologyEx(fore, fore, CV_MOP_CLOSE, element);

    //Removing shadows.
    threshold(fore, fore, 200, 255, THRESH_BINARY);

    return fore;
    
}

//Check if box is moving.
bool BlobDetector::checkBoxMoving(vector<Rect> * windows, Rect r0){
    vector<Rect> :: iterator itr = windows->begin();
    
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
    
    vector<Rect> :: iterator itr = windows->begin();
    
    while(itr!=windows->end()){
        Rect intersection = (r0 & *itr);
        float ratio = intersection.area() / float((*itr).area()) ;
        ratio = ratio * 100;
        
        if(ratio > 30){
            intersection += intersection.size();
            windows->erase(itr);
            windows->insert(itr,intersection);
        }
        
        itr++;
    }
}

// Using SimpleBlobDetector to get blobs.
Mat BlobDetector::detectBLOBS(){
    
    trackedWindows = vector<Rect>();
    
    SimpleBlobDetector::Params pDefaultBLOB;
    // This is default parameters for SimpleBlobDetector
    pDefaultBLOB.thresholdStep = 10;
    pDefaultBLOB.minThreshold = 10;
    pDefaultBLOB.maxThreshold = 220;
    pDefaultBLOB.minRepeatability = 2;
    pDefaultBLOB.minDistBetweenBlobs = 10;
    pDefaultBLOB.filterByColor = false;
    pDefaultBLOB.blobColor = 0;
    pDefaultBLOB.filterByArea = false;
    pDefaultBLOB.minArea = 25;
    pDefaultBLOB.maxArea = 5000;
    pDefaultBLOB.filterByCircularity = false;
    pDefaultBLOB.minCircularity = 0.9f;
    pDefaultBLOB.maxCircularity = (float)1e37;
    pDefaultBLOB.filterByInertia = false;
    pDefaultBLOB.minInertiaRatio = 0.1f;
    pDefaultBLOB.maxInertiaRatio = (float)1e37;
    pDefaultBLOB.filterByConvexity = false;
    pDefaultBLOB.minConvexity = 0.95f;
    pDefaultBLOB.maxConvexity = (float)1e37;
    // Descriptor array for BLOB
    vector<String> typeDesc;
    // Param array for BLOB
    vector<SimpleBlobDetector::Params> pBLOB;
    vector<SimpleBlobDetector::Params>::iterator itBLOB;
    
    // Color palette
    vector< Vec3b >  palette;
    for (int i = 0; i<65536; i++)
    {
        palette.push_back(Vec3b((uchar)rand(), (uchar)rand(), (uchar)rand()));
    }
    
    // This descriptor are going to be detect and compute BLOBS with 6 differents params
    // Param for first BLOB detector we want all
    typeDesc.push_back("BLOB");    // see http://docs.opencv.org/trunk/d0/d7a/classcv_1_1SimpleBlobDetector.html
    pBLOB.push_back(pDefaultBLOB);
    pBLOB.back().filterByArea = true;
    pBLOB.back().minArea = 1;
    pBLOB.back().maxArea = float(fore.rows*fore.cols);
    // Param for second BLOB detector we want area between 500 and 2900 pixels
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);
    pBLOB.back().filterByArea = true;
    pBLOB.back().minArea = 500;
    pBLOB.back().maxArea = 2900;
    // Param for third BLOB detector we want only circular object
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);
    pBLOB.back().filterByCircularity = true;
    // Param for Fourth BLOB detector we want ratio inertia
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);
    pBLOB.back().filterByInertia = true;
    pBLOB.back().minInertiaRatio = 0;
    pBLOB.back().maxInertiaRatio = (float)0.2;
    // Param for fifth BLOB detector we want ratio inertia
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);
    pBLOB.back().filterByConvexity = true;
    pBLOB.back().minConvexity = 0.;
    pBLOB.back().maxConvexity = (float)0.9;
    // Param for six BLOB detector we want blob with gravity center color equal to 0 bug #4321 must be fixed
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);
    pBLOB.back().filterByColor = true;
    pBLOB.back().blobColor = 0;
    
    itBLOB = pBLOB.begin();
    
    vector<double> desMethCmp;
    Ptr<Feature2D> b;
    String label;
    // Descriptor loop
    vector<String>::iterator itDesc;
    for (itDesc = typeDesc.begin(); itDesc != typeDesc.end(); itDesc++){
        vector<KeyPoint> keyImg1;
        if (*itDesc == "BLOB"){
            b = SimpleBlobDetector::create(*itBLOB);
            itBLOB++;
        }
        try
        {
            // We can detect keypoint with detect method
            vector<KeyPoint>  keyImg;
            vector<Rect>  zone;
            vector<vector <Point> >  region;
            Mat     desc, result(fore.rows, fore.cols, CV_8UC3);
            if (b.dynamicCast<SimpleBlobDetector>() != NULL)
            {
                Ptr<SimpleBlobDetector> sbd = b.dynamicCast<SimpleBlobDetector>();
                sbd->detect(fore, keyImg, Mat());
                drawKeypoints(fore, keyImg, result);
                int i = 0;
                for (vector<KeyPoint>::iterator k = keyImg.begin(); k != keyImg.end(); k++, i++){
                    //circle(result, k->pt, (int)k->size, palette[i % 65536]);
                    RotatedRect rect = RotatedRect((Point2f)k->pt, Size2f(k->size, k->size), 0);
                    
                    Rect r0 = rect.boundingRect();
                    
                    if(r0.area() > 200  && r0.area() < 1500 ){
                        checkWindowsOverlap(&trackedWindows, r0);
                        trackedWindows.push_back(r0);
                        rectangle(result, r0 , palette[i % 65536]);
                    }
                }
            }
            
            return result;
            
        }
        catch (Exception& e)
        {
            cout << "Feature : " << *itDesc << "\n";
            cout << e.msg << endl;
        }
    }
    
    return Mat();
}

// Getting BLOBS first version.
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
    vector<Point3f> :: iterator itl = points.begin();
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
        if(r0.area() > 200 /*&& r0.area() < 1200*/){
            checkWindowsOverlap(&trackedWindows, r0);
            trackedWindows.push_back(r0);
        }
    }
    
    return dst;
    
}

//Get moving object using ROI.
vector<Rect> BlobDetector::getMovingObjects(){
    
    vector<Rect> detectedWindows;
    vector<Rect> multipleROI;
    
    //denmark1.avi ROI.
    multipleROI.push_back(Rect(200,165,5,40));
    multipleROI.push_back(Rect(260,300,50,5));
//    multipleROI.push_back(Rect(380,130,50,5));
    
//    innout.mp4
//    multipleROI.push_back(Rect(200,165,5,40));
//    multipleROI.push_back(Rect(600,165,5,40));
    
    //nevada1.avi ROI.
    //Rect detectionROI = Rect(200,160,60,120);
    
    //denmark4.avi ROI.
    //Rect firstROI = Rect(240,100,5,120);
    
    // Draw multiple ROI
    vector<Rect>::iterator itw = multipleROI.begin();
    while(itw != multipleROI.end()){
        rectangle(frame, *itw, Scalar(0, 0, 255));
        itw++;
    }
    
    itw = multipleROI.begin();
    
    vector<Rect> :: iterator itr = trackedWindows.begin();

    while(itr!=trackedWindows.end()){
        while(itw != multipleROI.end()){
            Rect intersection = (*itw & *itr);
            if(intersection.area() > 0){
                float ratio = intersection.area() / float((*itr).area()) ;
                ratio = ratio * 100;
                
                if(ratio > 10){
                    //bool result = checkBoxMoving(&detectedWindows, *itr);
                    //if(!result){
                        detectedWindows.push_back(*itr);
                    //}
                }
            }
            itw++;
        }
        itw = multipleROI.begin();
        itr++;
    }

    
    return detectedWindows;
}


// Drawing tracked windows.
Mat BlobDetector::drawTrackedWindows(){
    vector<Rect> :: iterator itr = trackedWindows.begin();
    
    while(itr!=trackedWindows.end()){
        rectangle(frame, *itr, Scalar(255, 0, 0));
        Point2f center = Point2f((*itr).x + (*itr).width/2,  (*itr).y+ (*itr).height/2);
        circle(frame, center, 1, Scalar(255, 0, 0), 2, 8, 0);
        itr++;
    }
    
    return frame;
}

// Drawing detected windows.
Mat BlobDetector::drawDetectedWindows(vector<Rect> detectedWindows){
    vector<Rect> :: iterator itr = detectedWindows.begin();
    
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
