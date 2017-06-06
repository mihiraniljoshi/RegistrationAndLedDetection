//
//  HSV.hpp
//  Registration_and_LED_detection
//
//  Created by Mihir Joshi on 3/5/16.
//  Copyright © 2016 Mihir Joshi. All rights reserved.
//

#ifndef HSV_hpp
#define HSV_hpp

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

IplImage* convertImageRGBtoHSV(const IplImage *);

#endif /* HSV_hpp */
