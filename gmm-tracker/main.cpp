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
#include <stdio.h>

#include "blob_detector.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    
    VideoCapture cap;
    cap.open("/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/videos/denmark4.avi");
    
    Mat frame, output;
    
    BlobDetector blobDetector;
    
    namedWindow("Video Output");
    
    while(1){
        cap >> frame;
        
        if (frame.empty())
            break;
        
        blobDetector.getFore(frame);
        blobDetector.getBLOBS();
        output = blobDetector.drawTrackedWindows(frame);
        
        imshow("Video Output", output);
        
        int key = waitKey(1);

        if ( key == 27 ) break;
        
    }

}
