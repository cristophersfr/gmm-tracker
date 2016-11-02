//
//  classifier.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 4/9/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include "classifier.hpp"

Classifier::Classifier(){

    bicycleClassifier = \
    DPMDetector::create(vector<string>(1, "/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/dpm-models/bicycles.xml"));
    
    pedestrian.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    
    lastDetected = Rect();
    n_frames = 0;
    bicycle_hits = 0;
    pedestrian_hits = 0;
    
    isClassified = false;
    
}

void Classifier::update(Mat frame, Rect object){
    if(!frame.empty())
        n_frames++;
    
    frame.copyTo(this->frame);
    this->object = object;
}

Rect Classifier::createROI(Rect object){
    
    // Expand size of the rectangle to guarantee a better detection.
    Point2f center = Point2f((this->object).x + (this->object).width/2,  (this->object).y+ (this->object).height/2);
    RotatedRect rotatedObject = RotatedRect(center, Size(150,150), 0);
    Rect _object = rotatedObject.boundingRect();
    
    if(_object.x < 0 && _object.y < 0){
        _object.x = 0;
        _object.y = 0;
    } else if(_object.y < 0){
        _object.y = 0;
    } else if(_object.x < 0){
        _object.x = 0;
    }
    
    return _object;
}

Rect Classifier::adjustBox(Rect objectROI, Mat frameROI){
    
    Point2f center = Point2f(frameROI.cols/2, frameROI.rows/2);
    
    Rect box = RotatedRect(center, object.size()*4, 0).boundingRect();
    
    // TODO: What if the size of the box oversize the frame?
    if(box.x < 0 && box.y < 0){
        box.x = 0;
        box.y = 0;
    } else if(box.y < 0){
        box.y = 0;
    } else if(box.x < 0){
        box.x = 0;
    }
    
    return box;
}

Rect Classifier::readjustBox(Rect object, Rect objectROI){
    
    Point2f center = Point2f(((object).x + (object).width/2)*0.25 + objectROI.x,
                             ((object).y+ (object).height/2)*0.25 + objectROI.y);
    
    RotatedRect box = RotatedRect(center, object.size()/4, 0);
    
    return box.boundingRect();
}

int Classifier::detectBicycles(){
    
    if(!isClassified){
        // Vector with objects detects in the frame.
        vector<DPMDetector::ObjectDetection> ds;
        
        // Create ROI based on the object bounding box.
        Rect objectROI = createROI(this->object);
        
        // Create an object mask from its frame.
        Mat frameROI = Mat(frame, objectROI);
        //imwrite("frameROI_" + to_string(n_frames) + ".jpg", frameROI);
        
        // Upscaling the sample for improving the detection.
        resize(frameROI, frameROI, Size(), 4, 4);
        
        // Transform the object bounding box from frame to frameROI.
        Rect objectBox = adjustBox(objectROI, frameROI);
    
        // Detect the object.
        bicycleClassifier->detect(frameROI, ds);
        
        if(ds.size() > 0){
            vector<DPMDetector::ObjectDetection>::iterator itr = ds.begin();
            while(itr != ds.end()){
                
                DPMDetector::ObjectDetection detectedObject = *itr;
                Rect detectedRect = detectedObject.rect;
                
                // Drawing detection.
                rectangle(frameROI, detectedRect, Scalar(0,255,0));
                rectangle(frameROI, objectBox, Scalar(255,0,0));
                //cout << objectBox;
                
                // Securing the last detection.
                this->lastDetected = readjustBox(detectedRect, objectROI);
                
                // Check the overlap ratio.
                Rect intersection = objectBox & detectedRect;
                int objectArea = objectBox.area();
                
                if(intersection.area() > 0){
                    float ratio = intersection.area() / float(objectArea) ;
                    ratio = ratio * 100;
                    //cout << "Bicycle Ratio: " << ratio << endl;
                    if(ratio > 40){
                        //Updating object with detected object.
                        //this->object = readjustBox(detectedRect, objectROI);
                        //cout << "n# frames" << n_frames << endl;
                        bicycle_hits++;
                        //cout << "hits: " << hits << endl;
                        String imageName = "bicycle_sample_" + to_string(bicycle_hits) + ".jpg";
                        imwrite(imageName, frameROI);
                        // Saving sample.
                    }
                }
                
                itr++;
            }
        }
    }
    
    isClassified = true;
    
    if(bicycle_hits%10 == 0 && bicycle_hits!=0) bicycle_hits+=10;
    
    float score = bicycle_hits/(float)n_frames;
    
    cout << "Bicycle Score: " << score << endl;
    
    if(n_frames >= 10){
        if(score > 0.2){
            return 1;
        } else {
            isClassified = false;
            return -1;
        }
    } else {
        isClassified = false;
        return 0;
    }
}

int Classifier::detectPedestrians(){
    
    if(!isClassified){
        //Vector of objects found.
        vector<Rect> detectedObjects;
        
        // Create ROI based on the object bounding box.
        Rect objectROI = createROI(this->object);
        
        // Create an object mask from its frame.
        Mat frameROI = Mat(frame, objectROI);
        
        // Upscaling the sample for improving the detection.
        resize(frameROI, frameROI, Size(), 4, 4);
        
        // Transform the object bounding box from frame to frameROI.
        Rect objectBox = adjustBox(objectROI, frameROI);
        
        cvtColor( frameROI, frameROI, COLOR_BGR2GRAY );
    
        pedestrian.detectMultiScale(frameROI, detectedObjects);
        
        if( detectedObjects.size() > 0 ){
            vector<Rect>::iterator itw = detectedObjects.begin();
            while(itw!=detectedObjects.end()){
                Rect detectedRect = *itw;
                
                // Drawing detection.
                rectangle(frameROI, detectedRect, Scalar(0,255,0));
                rectangle(frameROI, objectBox, Scalar(255,0,0));
                //cout << objectBox;
                
                // Securing the last detection.
                this->lastDetected = readjustBox(detectedRect, objectROI);
                
                // Check the overlap ratio.
                Rect intersection = objectBox & detectedRect;
                int objectArea = objectBox.area();
                int detectedArea = detectedRect.area();
                
                if(intersection.area() > 0){
                    float ratio = intersection.area() / float(objectArea) ;
                    ratio = ratio * 100;
                    //cout << "Pedestrian Ratio: " << ratio << endl;
                    if(ratio > 40){
                        //Updating object with detected object.
                        pedestrian_hits++;
                        String imageName = "pedestrian_sample_" + to_string(pedestrian_hits) + ".jpg";
                        imwrite(imageName, frameROI);
                    }
                }
                
                itw++;
            }
            
        }
    }
    
    isClassified = true;
    
    if(pedestrian_hits%10 == 0 && pedestrian_hits!=0) pedestrian_hits+=10;
    
    float score = pedestrian_hits/(float)n_frames;
    
    cout << "Pedestrian Score: " << score << endl;
    
    if(n_frames >= 10){
        if(score > 0.4){
            return 1;
        } else {
            isClassified = false;
            return -1;
        }
    } else {
        isClassified = false;
        return 0;
    }
    
}

Rect Classifier::getObject(){
    return this->lastDetected;
}

int Classifier::getId(){
    return this->id;
}
