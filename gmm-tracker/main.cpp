//
//  main.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 2/16/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include <thread>
#include <stdio.h>
#include <semaphore.h>
#include <time.h>

#include "blob_detector.hpp"
#include "classifier.hpp"
#include "kcftracker/kcftracker.hpp"

Classifier classifier;

// Vector with each thread containing a tracker running.
vector<thread> threads;

// Vector with tracked windows at each frame.
vector<Rect> resultsWindows;

// Counting objects being tracked.
int num_results;

// General semaphore for avoiding frame being accessed while being changed.
sem_t * frameLock;

// Verifying if the objects is not being tracked already.
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
                // Ratio of overlapping.
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

// Thread function responsible for keep updating the tracker.
void runTracker(KCFTracker * tracker, Mat * frame){
    Rect * result = new Rect();
    int i = num_results;
    num_results++;
    resultsWindows.push_back(*result);
    while(!frame->empty()){
        sem_wait(frameLock);
        *result = tracker->update(*frame);
        resultsWindows[i] = *result;
        classifier.isBike(*frame, *result);
        rectangle( *frame, Point( result->x, result->y ),
                  Point( result->x+result->width, result->y+result->height),
                  Scalar( 0, 255, 255 ), 1, 8 );
        sem_post(frameLock);
    }
}

// Starting tracking and creating the thread responsible.
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
    
#ifdef HAVE_TBB // parallel computing
    cout << "Using TBB" << endl;
#endif
    
    Mat frame;
    
    // Change this if not Unix.
    sem_unlink("frameSync");
    frameLock = sem_open("frameSync", O_CREAT, 0700, 1);

    VideoCapture cap;
    cap.open("/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/videos/denmark1.avi");
    
//    int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
//    
//    Size S = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH)*2,
//                  (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));
//    
//    VideoWriter outputVideo;
//    outputVideo.open("/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/samples/denmark1-dual.avi",
//                     ex, cap.get(CV_CAP_PROP_FPS), S, true);
    
    // BlobDetector(int history, int nMixtures, bool detectShadows)
    BlobDetector blobDetector(120, 3, true);
    vector<Rect> objectsWindows;
    bool init = false;
    
    int num_frames = 0;
    num_results = 0;
    
    // Start and end times
    time_t start, end;
    
    // Start time
    time(&start);
    
    while(1){
        
        //Synchronize frame capture.
        sem_wait(frameLock);
//        for(int i=0; i < 2; i++){
            if(!cap.read(frame)){
                cout << "Could not load the frame." << endl;
                return -1;
            }
//            resize(frame, frame, Size(), 0.5, 0.5);
//        }
        sem_post(frameLock);
        
        if (frame.empty()){
            cout << "Could not load frame." << endl;
            break;
        }
        
        Mat fore = blobDetector.getFore(frame);
        
        // Get BLOBS.
        blobDetector.detectBLOBS();
        
        // Detect objects through intersection.
        objectsWindows = blobDetector.getMovingObjects();
        
        // Drawing output.
        Mat output;
        blobDetector.drawTrackedWindows();
        output = blobDetector.drawDetectedWindows(objectsWindows);
        
        //Call KCFTracker.
        if(objectsWindows.size() > 0 && !init){
            //cout << objectsWindows.size();
            trackObjects(objectsWindows, &frame);
            init = true;
        } else if(init){
            objectsWindows = checkTrackingWindows(objectsWindows);
            if(objectsWindows.size() > 0){
                trackObjects(objectsWindows, &frame);
            }
        }
        
        // Counting frames.
        num_frames++;
        
        // End Time
        time(&end);
        
        // Time elapsed
        double seconds = difftime (end, start);
        
        // Calculate frames per second
        double fps  = num_frames / seconds;
        
        string text = "FPS: " + to_string(int(round(fps)));
        string title_1 = "Motion Detection";
        string title_2 = "Object Tracking";
        putText(output, title_1, Point(10,15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0),2,8);
        putText(frame, title_2, Point(10,15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0),2,8);
        putText(frame, text, Point(10,35), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2,8);
        putText(frame, to_string(threads.size()), Point(10,55), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2,8);
        
        // Display images.
        sem_wait(frameLock);
//        imshow("Tracker Output", output);
//        imshow("Detection Output", frame);
        Mat video_output;
        hconcat(output, frame, video_output);
        imshow("Video Output", video_output);
        //outputVideo.write(video_output);
        //imshow("Foreground",fore);
        sem_post(frameLock);
        
        int key = waitKey(1);

        if ( key == 27 ) break;
        
    }
    
    // Joining unfinished threads.
    for(int i = 0; i < threads.size(); i++){
        if(threads[i].joinable()){
            threads[i].join();
        }
    }
    
    cap.release();
//    outputVideo.release();

}
