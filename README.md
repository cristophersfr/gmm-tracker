# Vision-based Pedestrian and Bicycle Detection and Counting System 

### Description: 
The tracker implements a Gaussian Mixture Model for Background Subtraction [1] and Kernelized Correlation Filters tracker [2] for multi-object using multithreading. The purpose of this work was to develop increment an Intelligent Transportation System (ITS), adding the capability of detecting and tracking bicycles and pedestrians. Though the implementation until now, only allow stable tracking according to the region of interest. This project was developed during an Independent Study class at UNLV Spring 2016, and continued by one more month after the classes, supported by Nevada Department of Transportation. 

#### This work was supported by NDOT under Cooperative Agreement No.14Q2-E1-6.

* [1] : <http://docs.opencv.org/3.1.0/d7/d7b/classcv_1_1BackgroundSubtractorMOG2.html>
* [2] : <https://github.com/joaofaro/KCFcpp>

### How to Execute: 

Requirements: 
  - Unix-based OS (Ubuntu and MacOS tested only)
  - OpenCV 3.1.0
  - TBB (optional)
  - OpenMP (optional)
  
```sh
$ git clone https://github.com/CristopherPK/gmm-tracker/
$ cd gmm-tracker
$ cmake .
$ make
$ ./GMMTracker videos/denmark1.avi
```
