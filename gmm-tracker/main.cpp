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

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    
    VideoCapture cap;
    cap.open("/Users/cristopher/Workspace/gmm-tracker/gmm-tracker/videos/denmark1.avi");
    
    Mat frame;
    
    Ptr<BackgroundSubtractorMOG2> bg = createBackgroundSubtractorMOG2();
    
    //bg.nmixtures = 3;
    //bg.bShadowDetection = true;
    //bg.nShadowDetection=127;
    //bg.fTau=0.3;
    
    bg->setNMixtures(3);
    bg->setDetectShadows(false);
    bg->setShadowValue(127);
    
    namedWindow("Video Output");
    
    Mat fore, back;
    
    while(1){
        cap >> frame;
        
        if (frame.empty())
            break;
        
        bg->apply(frame, fore);
        
        medianBlur(fore, fore, 3);
        
        imshow("Video Output", fore);
        
        int key = waitKey(1);

        if ( key == 27 ) break;
        
    }

}
