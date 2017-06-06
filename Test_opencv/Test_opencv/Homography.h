#include <stdlib.h>
#include <stdio.h>
#include <fstream> 
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>  
#include <time.h>  
#include <fstream>

using namespace cv; 
using namespace std;
using namespace cv::xfeatures2d;

std::vector<int> FindLEDAssociation(int LED_x[10],int LED_y[10],string Router_name,Mat img_scene1);