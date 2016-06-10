//
//  object.cpp
//  gmm-tracker
//
//  Created by Cristopher Freitas on 6/6/16.
//  Copyright © 2016 Cristopher Freitas. All rights reserved.
//

#include "object.hpp"

Object::Object(int id, Rect box, String description){
    this->id = id;
    this->box = box;
    //Validates this description.
    this->description = description;
}