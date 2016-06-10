//
//  object.hpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 6/6/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#ifndef object_hpp
#define object_hpp

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Object {
    int id;
    Rect box;
    String description;
    
public:
    Object(int id, Rect box, String description);
};

#endif /* object_hpp */
