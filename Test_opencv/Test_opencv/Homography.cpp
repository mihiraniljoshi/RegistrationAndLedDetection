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
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>  
#include <time.h>  
#include <fstream>
#include "HSV.hpp"

using namespace cv; 
using namespace std;
using namespace cv::xfeatures2d;

std::vector<int> FindLEDAssociation(int LED_x[],int LED_y[],string Router_name,Mat img_scene1) 
{
	std::vector<int> v(10);

	//Reading Template LED locations
	string line;

	ifstream LED_loc("./Router_Templates/"+Router_name+"/LEDloc.txt");
	if (LED_loc.is_open())
	{
		while ( getline (LED_loc,line) )
		{
			//cout << line << '\n';
		}
		LED_loc.close();
	}
	else cout << "Unable to open file";

	int LED_no_temp=0;
	int LED_x_temp[10],LED_y_temp[10];
	std::string tok1;
	int loc_count=0;
	int offset_x = 55, offset_y = 131;

	std::istringstream stream1(line);
	loc_count=0;
	while (stream1)
	{
		std::getline(stream1, tok1, ',');
		//std::cout << tok1 << std::endl;
		if(loc_count<10)
		{
			if(stoi(tok1)!=0)
			{
				LED_x_temp[loc_count] = (stoi(tok1)-offset_x)*4;
				LED_no_temp++;
			}

		}
		else
		{
			if(stoi(tok1)!=0)
			{
				LED_y_temp[loc_count-10] = (stoi(tok1)-offset_y)*4;
			}

		}
		loc_count++;
	}

	//Reading Template Image 
	string Img_temp_LOC = "./Router_Templates/"+Router_name+"/frame1.jpg";
	Mat img_object1 = imread(Img_temp_LOC.c_str(),0);

	Size size(270,480);
	Mat img_1;
	Mat img_2;
	resize(img_object1,img_1,size);
	resize(img_scene1,img_2,size);

	if( !img_1.data || !img_2.data )
	{ std::cout<< " --(!) Error reading images " << std::endl; return v; }

	//Loop through all possible combinations of LED locations to find the best fitting transform

	Mat T;
	float min_error=20000,error_mat[10];

	//Loop for 2nd LED locations
	std::stringstream s1;
	for(int i=0;i<LED_no_temp;i++)
	{
		s1<<i;
	}

	std::string s = s1.str(),best_s;
	std::sort(s.begin(), s.end());
	do {
		//Mapping of LED locations
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		//Push the LED locations in the list
		for(int i=0;i<LED_no_temp;i++)
		{
			obj.push_back(Point2f(LED_x_temp[i]/4,LED_y_temp[i]/4));
			scene.push_back(Point2f(LED_x[stoi(s.substr(i,1))]/4,LED_y[stoi(s.substr(i,1))]/4));
		}

		cv::Mat R = cv::estimateRigidTransform(obj,scene,true);

		if(R.dims>0)
		{
			// extend rigid transformation to use perspectiveTransform:
			cv::Mat H = cv::Mat(3,3,R.type());
			H.at<double>(0,0) = R.at<double>(0,0);
			H.at<double>(0,1) = R.at<double>(0,1);
			H.at<double>(0,2) = R.at<double>(0,2);

			H.at<double>(1,0) = R.at<double>(1,0);
			H.at<double>(1,1) = R.at<double>(1,1);
			H.at<double>(1,2) = R.at<double>(1,2);

			H.at<double>(2,0) = 0.0;
			H.at<double>(2,1) = 0.0;
			H.at<double>(2,2) = 1.0;

			//Mapping of LED locations
			std::vector<Point2f> inp;
			std::vector<Point2f> outp;
			for(int i=0;i<LED_no_temp;i++)
			{
				inp.push_back(Point2f(LED_x_temp[i]/4,LED_y_temp[i]/4));
			}
			perspectiveTransform( inp, outp, H);

			//Error Calculations
			float error_avg=0;
			for(int i=0;i<LED_no_temp;i++)
			{
				error_mat[i]=sqrt(pow((outp[i].x-LED_x[stoi(s.substr(i,1))]/4),2)+pow((outp[i].y-LED_y[stoi(s.substr(i,1))]/4),2));
				error_avg=error_avg+error_mat[i];
			}
			error_avg=error_avg/LED_no_temp;
			if(error_avg<min_error)
			{
				min_error=error_avg;
				T=H;
				best_s=s.c_str();
			}
		}
		//std::cout << s << '\n';
	} while(std::next_permutation(s.begin(), s.end()));

	Mat out_img;
	warpPerspective (img_2, out_img, T, img_2.size(),INTER_LINEAR + WARP_INVERSE_MAP);
	//warpAffine (img_2, out_img, R, img_2.size(),INTER_LINEAR + WARP_INVERSE_MAP);
	imshow("in",img_2);
	//Output Image
	imshow("out",out_img);

	//Mapping of LED locations
	std::vector<Point2f> inp;
	std::vector<Point2f> outp;
	for(int i=0;i<LED_no_temp;i++)
	{
		inp.push_back(cvPoint(LED_x_temp[i]/4,LED_y_temp[i]/4));
	}

	perspectiveTransform( inp, outp, T);

	for(int i=0;i<LED_no_temp;i++)
	{
		circle( img_2, outp[i], 10.0, Scalar( 250, 250, 250 ), 1, 8 );
	}
	imshow("Mapping",img_2);

	//LED mapping 
	int LED_map[10];
	for(int i=0;i<LED_no_temp;i++)
	{
		v[i]=stoi(best_s.substr(i,1));
	}
	cvWaitKey(1);
	return v;
}