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
#include <mutex>          // std::mutex

std::mutex mtx;           // mutex for critical section

#ifdef __linux__ 
    #include <fcntl.h>
#endif

#include "blob_detector.hpp"
#include "classifier.hpp"
#include "kcftracker/kcftracker.hpp"

vector<thread> threads;
vector<thread> trackerThreads;
vector<thread> detectionThreads;

// Vector with tracked windows at each frame.
vector<Rect> resultsWindows;
vector<Rect> bicycles;

// Counting objects being tracked.
int id;
int num_results;
int num_bicycles;
int num_pedestrians;

// General semaphore for avoiding frame being accessed while being changed.
sem_t * frameLock;

void writeInfo(double fps, Mat * frame, Mat * output);
vector<Rect> checkWindows(vector<Rect> source, vector<Rect> windows);
void trackObjects(vector<Rect> objects, Mat * frame);
void runTracker(KCFTracker * tracker, Mat * frame);
void detectObjects(vector<Rect> objects, Mat frame);
void runClassifier(Classifier * classifier, Mat frame);

int main(int argc, char ** argv){
    
#ifdef HAVE_TBB // parallel computing
    cout << "Using TBB" << endl;
#endif
    
    int id = 0;
    
    Mat frame;
    
    // Change this if not Unix.
    sem_unlink("frameSync");
    frameLock = sem_open("frameSync", O_CREAT, 0700, 1);

    /* 
     TO DO:
     
     1 - Get video path using argument.
     2 - Insert decision of writing video output or not.
     
     */
    
    VideoCapture cap;
//    cap.open("/home/cristopher/workspace/gmm-tracker/gmm-tracker/videos/denmark1.avi");
    cap.open("/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/videos/denmark1.avi");
//    cap.open("/home/cristopher/Workspace/gmm-tracker/gmm-tracker/videos/denmark1.avi");
    
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
//        sem_wait(frameLock);
        mtx.lock();
//        for(int i=0; i < 2; i++){
            if(!cap.read(frame)){
                cout << "Could not load the frame." << endl;
                return -1;
            }
//            resize(frame, frame, Size(), 0.5, 0.5);
//        }
        mtx.unlock();
//        sem_post(frameLock);
        
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
            detectObjects(objectsWindows, frame);
//            trackObjects(objectsWindows, &frame);
            init = true;
        } else if(init){
            objectsWindows = checkWindows(bicycles, objectsWindows);
            if(objectsWindows.size() > 0){
                //trackObjects(objectsWindows, &frame);
                detectObjects(objectsWindows, frame);
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
        
        writeInfo(fps, &frame, &output);
        
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

void writeInfo(double fps, Mat * frame, Mat * output){
    string text = "FPS: " + to_string(int(round(fps)));
    string title_1 = "Motion Detection";
    string title_2 = "Object Tracking";
    putText(*output, title_1, Point(10,15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0),2,8);
    putText(*frame, title_2, Point(10,15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0),2,8);
    putText(*frame, text, Point(10,35), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2,8);
    putText(*frame, "Bicycles: " + to_string(num_bicycles), Point(10,55), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2,8);
}

// Verifying if the objects is not being tracked already.
vector<Rect> checkWindows(vector<Rect> source, vector <Rect> windows){
    vector<Rect> :: const_iterator itw = windows.begin();
    vector<Rect> :: const_iterator itr = source.begin();
    vector<Rect> results;
    
    //cout << resultsWindows.size() << endl
    bool overlap_flag = false;
    
    while(itw != windows.end()){
        while(itr != source.end()){
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

// Thread function responsible for keep updating the tracker.
void runTracker(KCFTracker * tracker, Mat * frame){
    //cout << "Calling new Thread" << endl;
    Rect * result = new Rect();
    int i = num_results;
    num_results++;
    resultsWindows.push_back(*result);
    while(!frame->empty()){
        //cout << "Locking thread" << endl;
        sem_wait(frameLock);
        //cout << "Releasing thread" << endl;
        *result = tracker->update(*frame);
        resultsWindows[i] = *result;
        rectangle( *frame, Point( result->x, result->y ),
                  Point( result->x+result->width, result->y+result->height),
                  Scalar( 0, 255, 255 ), 1, 8 );
        sem_post(frameLock);
    }
}

//
void detectObjects(vector<Rect> objects, Mat frame){
    vector<Rect> :: const_iterator itr = objects.begin();
    while(itr!=objects.end()){
        Classifier * classifier = new Classifier(id, frame, *itr);
        id++;
        bicycles.push_back(*itr);
        classifier->detectBicycles();
        detectionThreads.push_back(thread (runClassifier, classifier, frame));
        itr++;
    }
}

void runClassifier(Classifier * classifier, Mat frame){
    while(!frame.empty()){
        mtx.lock();
        classifier->update(frame);
        classifier->detectBicycles();
        mtx.unlock();
    }
}
