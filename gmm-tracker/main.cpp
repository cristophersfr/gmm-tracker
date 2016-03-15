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
#include <time.h>

#include "blob_detector.hpp"
#include "kcftracker/kcftracker.hpp"

using namespace std;
using namespace cv;

vector<thread> threads;

vector<Rect> resultsWindows;

int num_results;

sem_t * frameLock;

//Verifying if the objects is not being tracked already.
vector<Rect> checkTrackingWindows(vector <Rect> windows){
    vector<Rect> :: const_iterator itw = windows.begin();
    vector<Rect> :: const_iterator itr = resultsWindows.begin();
    vector<Rect> results;
    
    //cout << resultsWindows.size() << endl;
    
    bool overlap_flag = false;
    
    while(itw != windows.end()){
        while(itr != resultsWindows.end()){
            Rect intersection = (*itr & *itw);
            int rect_area = (*itr).area();
            
            if(intersection.area() > 0){
                float ratio = intersection.area() / float(rect_area) ;
                ratio = ratio * 100;
                //cout << ratio << endl;
                //Ratio of overlapping.
                if(ratio < 10){
                    overlap_flag = false;
                } else {
                    overlap_flag = true;
                    break;
                }
            } else {
                overlap_flag = false;
            }
            
            itr++;
        }
        
        if(!overlap_flag){
            results.push_back(*itw);
        }
        
        itw++;
        
        itr = resultsWindows.begin();
    }
    
    return results;
}

//Thread function responsible for keep updating the tracker.
void runTracker(KCFTracker * tracker, Mat * frame){
    Rect * result = new Rect();
    int i = num_results;
    num_results++;
    resultsWindows.push_back(*result);
//    cout << resultsWindows.front() << endl;
    while(!frame->empty()){
        sem_wait(frameLock);
        *result = tracker->update(*frame);
        resultsWindows[i] = *result;
        rectangle( *frame, Point( result->x, result->y ),
                  Point( result->x+result->width, result->y+result->height),
                  Scalar( 0, 255, 255 ), 1, 8 );
        sem_post(frameLock);
    }
}

//Starting tracking and creating the thread responsible.
void trackObjects(vector<Rect> objects, Mat * frame){
    vector<Rect> :: const_iterator itr = objects.begin();
    int i = 0;
    while(itr!=objects.end()){
        KCFTracker * tracker = new KCFTracker();
        sem_wait(frameLock);
        tracker->init(*itr, *frame);
        threads.push_back(thread (runTracker, tracker, frame));
        //threads[i].detach();
        sem_post(frameLock);
        i++;
        itr++;
    }
}

int main(int argc, char** argv) {
    
    Mat frame;
    
    sem_unlink("frameSync");
    frameLock = sem_open("frameSync", O_CREAT, 0700, 1);

    VideoCapture cap;
    cap.open("/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/videos/denmark.mkv");
    
    Mat output;
    BlobDetector blobDetector;
    vector<Rect> objectsWindows;
    bool init = false;
    
    namedWindow("Video Output");
    
    int num_frames = 0;
    num_results = 0;
    
    // Start and end times
    time_t start, end;
    
    // Start time
    time(&start);
    
    while(1){
        
        //Synchronize frame capture.
        sem_wait(frameLock);
        cap.read(frame);
        sem_post(frameLock);
        
        if (frame.empty())
            break;
        
        output = frame;
        
        //Get foreground.
        Mat fore = blobDetector.getFore(frame);
        
        //Training period.
        if(num_frames > 400){
        
            //Get BLOBS.
            //Very expensive operation.
            output = blobDetector.getBLOBS();
            
            //Detect objects through intersection.
            objectsWindows = blobDetector.getMovingObjects();
            
            //Drawing output.
            output = blobDetector.drawTrackedWindows();
            output = blobDetector.drawDetectedWindows(objectsWindows);
            
            //Call KCFTracker.
            if(objectsWindows.size() > 0 && !init){
                //cout << objectsWindows.size();
                trackObjects(objectsWindows, &frame);
                init = true;
            }
            else if(init){
                objectsWindows = checkTrackingWindows(objectsWindows);
                if(objectsWindows.size() > 0){
                    trackObjects(objectsWindows, &frame);
                }
            }
            
        }
        
        //Counting frames.
        num_frames++;
        
        // End Time
        time(&end);
        
        // Time elapsed
        double seconds = difftime (end, start);
        //cout << "Time taken : " << seconds << " seconds" << endl;
        
        // Calculate frames per second
        double fps  = num_frames / seconds;
        
        string text = "FPS: " + to_string(int(round(fps)));
        
        putText(output, text, Point(10,15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2,8);
        
        putText(output, to_string(threads.size()), Point(10,35), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2,8);
        
        imshow("Video Output", output);
        //imshow("Foreground", fore);
        
        int key = waitKey(1);

        if ( key == 27 ) break;
        
    }
    
    for(int i = 0; i < threads.size(); i++){
        if(threads[i].joinable()){
            threads[i].join();
        }
    }
    
    cap.release();

}
