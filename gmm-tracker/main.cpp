//
//  main.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 2/16/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include <thread>
#include <fstream>
#include <stdio.h>
#include <semaphore.h>
#include <time.h>

#ifdef __linux__
#include <fcntl.h>
#endif

#include "blob_detector.hpp"
#include "classifier.hpp"
#include "kcftracker/kcftracker.hpp"

//#define HAVE_TBB TRUE

ofstream dataOutput;

// Vector with each thread containing a tracker running.
vector<thread> trackerThreads;

// Vector with tracked windows at each frame.
vector<Rect> resultsWindows;

// Counting objects being tracked.
int num_results;
int num_frames;

// Object id;
int id;

// General semaphore for avoiding frame being accessed while being changed.
sem_t * frameLock;

void drawPath(vector<Point2f> path, Mat frame, Scalar color){
    vector<Point2f> :: const_iterator itp = path.begin();
    while(itp!=path.end()){
        circle(frame, *itp, 1, color);
        itp++;
    }
}

// Verifying if the objects is not being tracked already.
vector<Rect> checkTrackingWindows(vector <Rect> windows){
    vector<Rect> :: const_iterator itw = windows.begin();
    vector<Rect> :: const_iterator itr = resultsWindows.begin();
    vector<Rect> results;

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
        
        if(!overlap_flag) results.push_back(*itw);
        
        itw++;
        itr = resultsWindows.begin();
    }
    
    return results;
}

// This packet will keep the persistence between tracker and classifier.
struct Packet {
    // Semaphore reference for synchronizing both threads.
    sem_t * sync_signal;
    // Flag for warning the tracker about detection result.
    bool flag;
    // Flag for warning the tracker to destroy itself.
    bool destroy;
    // Reference for the bounding box object.
    Rect * box;
    // Frame reference
    Mat * frame;
    // Object description
    String description;
}; typedef Packet Packet;

bool overlaps(Rect r1, Rect r2){
    // Check the overlap ratio.
    Rect intersection = r1 & r2;
    Rect dest;
    (r1.area() > r2.area())?(dest = r2) : (dest = r1);
    if(intersection.area() > 0){
        float ratio = intersection.area() / float(dest.area()) ;
        ratio = ratio * 100;
        return (ratio > 40) ? (true) : (false);
    }
    return false;
}

void runClassifier(Packet * p){
    Classifier * classifier = new Classifier();
    while(p != NULL){ // WARNING: This comparison can cause problems..
        
        // Waiting for tracker release the detection.
        sem_wait(p->sync_signal);
        // Update the resources.
        classifier->update(*p->frame, *p->box);
        // Try to detect.
        if(classifier->detectBicycles() == 1){
            p->description = "bicycle";
        } else if(classifier->detectPedestrians() == 1){
            p->description = "pedestrian";
        } else if(classifier->detectBicycles() < 0 &&
                  classifier->detectPedestrians() < 0){
            //*p->box = classifier->getObject();
            //if(p->box->area() > 0){
            //    p->flag = true;
            //} else {
                p->destroy = true;
                cout << "Exiting thread" << endl;
                sem_post(p->sync_signal);
                pthread_exit(0);
            //}
        }
        // Release tracker to continue its job.
        sem_post(p->sync_signal);
        
    }
    
    pthread_exit(0);
}

// Thread function responsible for keep updating the tracker.
void runTracker(KCFTracker * tracker, Mat * frame){
    vector<Point2f> path;
    // General semaphore for synchronizing threads.
    sem_t * sync;
    sem_unlink("detectSync");
    sync = sem_open("detectSync", O_CREAT, 0700, 0);
    
    Scalar color = Scalar(rand()&255, rand()&255, rand()&255);
    
    Rect * result = new Rect();
    int i = num_results;
    num_results++;
    resultsWindows.push_back(*result);
    
    Packet * p = new Packet();
    p->sync_signal = sync;
    p->box = result;
    p->frame = frame;
    p->flag = (bool *) false;
    p->destroy = (bool *) false;
    
    thread t(runClassifier, p);
    
    while(!frame->empty()){
        sem_wait(frameLock);
        *result = tracker->update(*frame);
        Point2f center = Point2f((*result).x + (*result).width/2,  (*result).y+ (*result).height/2);
        path.push_back(center);
        resultsWindows[i] = *result;
        p->box = result;
        sem_post(sync);
        sem_wait(sync);
        // Trying to recover the tracker path.
        if(p->flag) tracker->init(*p->box, *frame);
        // Destroying the tracker if not detected.
        if(p->destroy){
            sem_post(frameLock);
            delete tracker;
            delete p;
            num_results--;
            pthread_exit(0);
        }
        rectangle( *frame, Point( result->x, result->y ),
                  Point( result->x+result->width, result->y+result->height),
                  Scalar( 0, 255, 255 ), 1, 8 );
        drawPath(path, *frame, color);
        sem_post(frameLock);
    }
    
    pthread_exit(0);
}

// Starting tracking and creating the thread responsible.
void trackObjects(vector<Rect> objects, Mat * frame){
    vector<Rect> :: const_iterator itr = objects.begin();
    while(itr!=objects.end()){
        KCFTracker * tracker = new KCFTracker();
        //sem_wait(frameLock);
        Point2f center = Point2f((*itr).x + (*itr).width/2,  (*itr).y+ (*itr).height/2);
        string line = to_string(id) + ";" + "undef;" + to_string(num_frames) + ";" + to_string(center.x) + ";" + to_string(center.y);
        id++;
        dataOutput << line << endl;
        dataOutput.flush();
        tracker->init(*itr, *frame);
        // WARNING: The tracker can happen to get this "resource" first than the frame capture.
        trackerThreads.push_back(thread (runTracker, tracker, frame));
        sem_post(frameLock);
        itr++;
    }
}

int main(int argc, char** argv) {
    
#ifdef HAVE_TBB // parallel computing
    cout << "Using TBB" << endl;
#endif
    
    Mat frame;
    id = 0;
    
    // Opening file for saving data.
    dataOutput.open("data.csv");
    dataOutput << "sep=;" << endl;
    string header = "id;label;timestamp;x;y";
    dataOutput << header << endl;
    dataOutput.flush();
    
    // Initializing semaphore for frame sync.
    sem_unlink("frameSync");
    frameLock = sem_open("frameSync", O_CREAT, 0700, 1);
    
    // Opening video file
    VideoCapture cap;
    cap.open("/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/videos/denmark1.avi");
    
    // Writing video file.
    int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));

    Size S = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH)*2,
                  (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));

    VideoWriter outputVideo;
    outputVideo.open("/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/samples/output.avi",
                     ex, cap.get(CV_CAP_PROP_FPS)/3, S, true);
    
    
    // BlobDetector(int history, int nMixtures, bool detectShadows)
    BlobDetector blobDetector(120, 3, false);
    vector<Rect> objectsWindows;
    bool init = false;
    
    num_frames = 0;
    num_results = 0;
    
    // Start and end times
    time_t start, end;
    
    // Start time
    time(&start);
    
    while(1){
        
        //Synchronize frame capture.
        sem_wait(frameLock);
        for(int i=0; i < 2; i++){
            if(!cap.read(frame)){
                cout << "Could not load the frame." << endl;
                break;
            }
            resize(frame, frame, Size(), 0.5, 0.5);
        }
    
        for(int i = 0; i < trackerThreads.size(); i++){
            sem_post(frameLock);
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
        putText(frame, to_string(num_results), Point(10,55), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2,8);
        
        // Display images.
        for(int i = 0; i < trackerThreads.size(); i++){
            sem_wait(frameLock);
        }
        
        Mat video_output;
        hconcat(output, frame, video_output);
        imshow("Video Output", video_output);
        outputVideo.write(video_output);
        sem_post(frameLock);
        
        int key = waitKey(1);
        
        if ( key == 27 ) break;
        
    }
    
    // Detaching unfinished threads.
    for(int i = 0; i < trackerThreads.size(); i++){
        if(trackerThreads[i].joinable()){
            trackerThreads[i].detach();
        }
    }
    
    cap.release();
    outputVideo.release();
    dataOutput.close();
    
}
