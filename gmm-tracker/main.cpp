//
//  main.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 2/16/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <thread>
#include <stdio.h>
#include <semaphore.h>

#include "blob_detector.hpp"
#include "kcftracker/kcftracker.hpp"

using namespace std;
using namespace cv;

vector<thread> threads;
vector<Rect> resultsWindows;

int nResults;

static Mat frame;

sem_t * frameLock;

//Verifying if the objects is not being tracked already.
vector<Rect> checkTrackingWindows(vector <Rect> windows){
    vector<Rect> :: const_iterator itw = windows.begin();
    vector<Rect> :: const_iterator itr = resultsWindows.begin();
    vector<Rect> results;
    
    while(itw!=windows.end()){
        while(itr!= resultsWindows.end()){
            Rect intersection = (*itr & *itw);
            int rect_area = (*itr).area();
            if(rect_area > 0){
                int ratio = intersection.area() / rect_area ;
                ratio = ratio * 100;
                //Ratio of overlapping.
                if(ratio < 40){
                    results.push_back(*itw);
                }
            }
            itr++;
        }
        itw++;
    }
    
    return results;
}

//Thread function responsible for keep updating the tracker.
void runTracker(KCFTracker * tracker){
    Rect result;
    resultsWindows.push_back(result);
    nResults++;
    while(!frame.empty()){
        sem_wait(frameLock);
        result = tracker->update(frame);
        //May happen a concurrency here.
        resultsWindows[nResults] = result;
        rectangle( frame, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 0, 255, 255 ), 1, 8 );
        sem_post(frameLock);
    }
}

//Starting tracking and creating the thread responsible.
void trackObjects(vector<Rect> objects){
    vector<Rect> :: const_iterator itr = objects.begin();
    int i = 0;
    while(itr!=objects.end()){
        KCFTracker * tracker = new KCFTracker();
        cout << &itr << endl;
        cout << "creating thread" << endl;
        tracker->init(*itr, frame);
        threads.push_back(thread (runTracker, tracker));
        threads[i].detach();
        i++;
        itr++;
    }
}

int main(int argc, char** argv) {
    
    sem_unlink("frameSync");
    frameLock = sem_open("frameSync", O_CREAT, 0700, 1);
    
    VideoCapture cap;
    cap.open("/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/videos/denmark4.avi");
    
    Mat output;
    BlobDetector blobDetector;
    vector<Rect> objectsWindows;
    bool init = false;
    namedWindow("Video Output");
    
    int nFrames = 0;
    nResults = 0;
    
    while(1){
        
        sem_wait(frameLock);
        cap >> frame;
        sem_post(frameLock);
        
        if (frame.empty())
            break;
        
        blobDetector.getFore(frame);
        blobDetector.getBLOBS();
        objectsWindows = blobDetector.getMovingObjects();
        output = blobDetector.drawTrackedWindows();
        output = blobDetector.drawDetectedWindows();
        
        if(objectsWindows.size() > 1 && !init){
            cout << objectsWindows.size();
            trackObjects(objectsWindows);
            init = true;
        }
        else if(init && (nFrames%10 == 0)){
            objectsWindows =  checkTrackingWindows(objectsWindows);
            if(objectsWindows.size() > 0){
                cout << "entrou aqui" << endl;
                trackObjects(objectsWindows);
            }
        }
        
        nFrames++;
                  
        imshow("Video Output", output);
        
        int key = waitKey(1);

        if ( key == 27 ) break;
        
    }

}
