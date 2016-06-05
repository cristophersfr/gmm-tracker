//
//  classifier.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 4/9/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include "classifier.hpp"

Classifier::Classifier(int id, Mat frame, Rect object){
    this->id = id;
    frame.copyTo(this->frame);
    this->object = object;
    
    bicycleClassifier = \
    DPMDetector::create(vector<string>(1, "/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/dpm-models/bicycles.xml"));
    
    pedestrian.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    
    n_samples = 0;
    total_ratio = 0;
    
    isClassified = false;
    result = false;
    
}

void Classifier::update(Mat frame){
    frame.copyTo(this->frame);
}

Rect Classifier::createROI(Rect object){
    
    // Expand size of the rectangle to guarantee a better detection.
    Point2f center = Point2f((this->object).x + (this->object).width/2,  (this->object).y+ (this->object).height/2);
    RotatedRect _object = RotatedRect(center, Size(150,150), 0);
    
//    _object = _object - Point(_object.width/2,_object.height/2);
    
//    if(_object.x < 0 && _object.y < 0){
//        _object.x = 0;
//        _object.y = 0;
//    } else if(_object.y < 0){
//        _object.y = 0;
//    } else if(_object.x < 0){
//        _object.x = 0;
//    }
    
    return _object.boundingRect();
}

Rect Classifier::adjustBox(Rect objectROI, Mat frameROI){
    
    Point2f center = Point2f(frameROI.cols/2, frameROI.rows/2);
    
    Rect box = RotatedRect(center, object.size()*4, 0).boundingRect();
    
    // TODO: What if the size of the box oversize the frame?
//    if(box.x < 0 && box.y < 0){
//        box.x = 0;
//        box.y = 0;
//    } else if(box.y < 0){
//        box.y = 0;
//    } else if(box.x < 0){
//        box.x = 0;
//    }
    
    return box;
}

Rect Classifier::readjustBox(Rect object, Rect objectROI){
    
    Point2f center = Point2f(((object).x + (object).width/2)*0.25 + objectROI.x,
                             ((object).y+ (object).height/2)*0.25 + objectROI.y);
    
    RotatedRect box = RotatedRect(center, object.size()/4, 0);
    
    return box.boundingRect();
}

bool Classifier::detectBicycles(){
    
    if(!frame.empty())
        n_frames++;
    
    // Vector with objects detects in the frame.
    vector<DPMDetector::ObjectDetection> ds;
    
    // Create ROI based on the object bounding box.
    Rect objectROI = createROI(this->object);
    
    // Create an object mask from its frame.
    Mat frameROI = Mat(frame, objectROI);
    
    // Upscaling the sample for improving the detection.
    resize(frameROI, frameROI, Size(), 4, 4);
    
    // Transform the object bounding box from frame to frameROI.
    Rect objectBox = adjustBox(objectROI, frameROI);
    
//    rectangle(frameROI, objectBox, Scalar(255,0,0));
//    String imageName = to_string(id) + "_sample_" + to_string(n_samples) + ".jpg";
//    imwrite(imageName, frameROI);
    
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
            cout << objectBox;
            
            // Check the overlap ratio.
            Rect intersection = objectBox & detectedRect;
            int objectArea = objectBox.area();
            
            if(intersection.area() > 0){
                //Updating object with detected object.
                this->object = readjustBox(detectedRect, objectROI);
                float ratio = intersection.area() / float(objectArea) ;
                ratio = ratio * 100;
                cout << ratio << endl;
                total_ratio = total_ratio + ratio;
                n_samples++;
                
                // Saving sample.
                String imageName = to_string(id) + "_sample_" + to_string(n_samples) + ".jpg";
                imwrite(imageName, frameROI);
            }

//            if(n_samples == 10){
//                isClassified = true;
//                float score;
//                score = total_ratio/float(n_samples);
//                cout << "Total(Ratio): " << total_ratio << endl;
//                cout << "Mean(Ratio): " << score << endl;
//                if( score > 40){
//                    result = true;
//                    return result;
//                } else {
//                    return false;
//                }
//            }
            
            // Detection score.
            //cout << detectedObject.score << endl;
            
            itr++;
        }
    }
    
    return false;
}

bool Classifier::detectPedestrians(){
    
    //Vector of objects found.
    vector<Rect> detectedObjects;
    
    Rect object = Rect(this->object);
    
    // Expand size of the rectangle to guarantee a better detection.
    object += object.size();
    object += object.size();
    object = object - Point(object.width/2,object.height/2);
    
    if(object.x < 0 && object.y < 0){
        object.x = 0;
        object.y = 0;
    } else if(object.y < 0){
        object.y = 0;
    } else if(object.x < 0){
        object.x = 0;
    }
    
    // Create an object mask from its frame.
    Mat frameROI = Mat(frame, object);
    
    cvtColor( frameROI, frameROI, COLOR_BGR2GRAY );
    
    resize(frameROI, frameROI, Size(), 4, 4);
    
    pedestrian.detectMultiScale(frameROI, detectedObjects);
    
    if( detectedObjects.size() > 0 ){
        vector<Rect>::iterator itw = detectedObjects.begin();
        while(itw!=detectedObjects.end()){
            
            rectangle(frameROI, *itw, Scalar(0,255,0));
            
            String imageName = "sample_" + to_string(n_samples) + ".jpg";
            n_samples++;
            imwrite(imageName, frameROI);
            itw++;
        }
    }
    
    return false;
    
}

