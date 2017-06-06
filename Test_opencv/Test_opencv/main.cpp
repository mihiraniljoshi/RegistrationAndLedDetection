//
//  main.cpp
//  OpenCV_test
//
//  Created by Mihir Joshi on 12/24/15.
//  Copyright © 2015 Mihir Joshi. All rights reserved.
//

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "HSV.hpp"
#include "dirent.h"
#include "Homography.h"

using namespace std;
using namespace cv;

int matching =1;
int RANSAC_flag = 1;

//Sort the ROI array and send index
std::vector<int> Sort_ROI(std::vector<int> x)
{
	std::vector<int> y(x.size());
	std::size_t n(0);
	std::generate(std::begin(y), std::end(y), [&]{ return n++; });

	std::sort(  std::begin(y),
		std::end(y),
		[&](int i1, int i2) { return x[i1] < x[i2]; } );
	return(y);

}

int main(int argc, const char * argv[]) {
	// insert code here...


	// Open LED loation text file

	ifstream LED_loc("Test_data/LEDloc1.txt");
	string line;
	if (LED_loc.is_open())
	{
		while ( getline (LED_loc,line) )
		{
			//cout << line << '\n';
		}
		LED_loc.close();
	}
	else cout << "Unable to open file";

	int LED_no=0;
	int LED_x[10],LED_y[10];
	int offset_x = 55, offset_y = 131;

	std::istringstream stream(line);
	std::string tok1;
	int loc_count=0;
	while (stream)
	{
		std::getline(stream, tok1, ',');
		//std::cout << tok1 << std::endl;
		if(loc_count<10)
		{
			if(stoi(tok1)!=0)
			{
				LED_x[loc_count] = (stoi(tok1)-offset_x)*4;
				LED_no++;
			}

		}
		else
		{
			if(stoi(tok1)!=0)
			{
				LED_y[loc_count-10] = (stoi(tok1)-offset_y)*4;
			}

		}
		loc_count++;
	}

	//Calculation of ROI size based on average LED seperation
	int ROI_size=0;
	for(int i=0;i<LED_no-1;i++)
	{
		ROI_size = ROI_size + sqrt((double)(((LED_x[i]-LED_x[i+1])*(LED_x[i]-LED_x[i+1]))+((LED_y[i]-LED_y[i+1])*(LED_y[i]-LED_y[i+1]))));
	}
	ROI_size = (ROI_size/(LED_no-1))*0.8;
	//Open video
	string filename = "Test_data/video1.mov";
	VideoCapture capture(filename);
	Mat frame1,frame1_rot,frame1_alligned,frame1_gray,frame2_gray,frame1_gray_aligned,frame1_gray_crop,frame1_alligned_HSV;
	IplImage *imHSV,*frame1_alligned_IPL;
	//Mat preview_frame;
	Mat H,S,V;
	Size size(270,480);
	vector<Mat> channels(3);

	if( !capture.isOpened() )
		throw "Error when reading steam_avi";

	//Read and Process subsequent frames
	int frame_counter = -1;
	int LED_map[10][61] = {0};
	int LED_map_filtered[10][61] = {0};
	for( int f=0;f<60 ;f++ )
	{
		capture >> frame1_rot;

		transpose(frame1_rot, frame1);  
		flip(frame1, frame1,1);

		frame_counter++;
		cout<<frame_counter<<endl;
		cvtColor(frame1, frame1_gray, CV_BGR2GRAY);
		resize(frame1_gray,frame1_gray_crop,size);

		if(frame1.empty())
			break;

		//Find out the LED location association with the data stored in database
		if(RANSAC_flag==1)
		{
			string Router_Name = "home";
			if(frame_counter==1)
			{
				std::vector<int> LED_asso = FindLEDAssociation(LED_x,LED_y,Router_Name,frame1_gray);
				int LED_x_temp[10]={0},LED_y_temp[10]={0};
				for(int i=0;i<LED_no;i++)
				{
					LED_x_temp[i]=LED_x[i];
					LED_y_temp[i]=LED_y[i];
				}
				for(int i=0;i<LED_no;i++)
				{
					LED_x[i]=LED_x_temp[LED_asso[i]];
					LED_y[i]=LED_y_temp[LED_asso[i]];
				}
			}
		}
		//Loop for image transform
		if(frame_counter>0)
		{
			// Define the motion model
			const int warp_mode = MOTION_EUCLIDEAN;

			// Set a 2x3 or 3x3 warp matrix depending on the motion model.
			Mat warp_matrix;

			// Initialize the matrix to identity
			if ( warp_mode == MOTION_HOMOGRAPHY )
				warp_matrix = Mat::eye(3, 3, CV_32F);
			else
				warp_matrix = Mat::eye(2, 3, CV_32F);

			// Specify the number of iterations.
			int number_of_iterations = 50;

			// Specify the threshold of the increment
			// in the correlation coefficient between two iterations
			double termination_eps = 1e-4;

			// Define termination criteria
			TermCriteria criteria (TermCriteria::COUNT+TermCriteria::EPS, number_of_iterations, termination_eps);

			// Run the ECC aSorithm. The results are stored in warp_matrix.
			findTransformECC(
				frame2_gray,
				frame1_gray_crop,
				warp_matrix,
				warp_mode,
				criteria
				);

			//Warping cropped frame
			if (warp_mode != MOTION_HOMOGRAPHY)
				// Use warpAffine for Translation, Euclidean and Affine
				warpAffine(frame1_gray_crop, frame1_gray_aligned, warp_matrix, frame1_gray_crop.size(), INTER_LINEAR + WARP_INVERSE_MAP);
			else
				// Use warpPerspective for Homography
				warpPerspective (frame1_gray_crop, frame1_gray_aligned, warp_matrix, frame1_gray_crop.size(),INTER_LINEAR + WARP_INVERSE_MAP);

			//Warping original frame
			warp_matrix.at<float>(0,2) = warp_matrix.at<float>(0,2) * 4;
			warp_matrix.at<float>(1,2) = warp_matrix.at<float>(1,2) * 4;

			if (warp_mode != MOTION_HOMOGRAPHY)
				// Use warpAffine for Translation, Euclidean and Affine
				warpAffine(frame1, frame1_alligned, warp_matrix, frame1.size(), INTER_LINEAR + WARP_INVERSE_MAP);
			else
				// Use warpPerspective for Homography
				warpPerspective (frame1, frame1_alligned, warp_matrix, frame1.size(),INTER_LINEAR + WARP_INVERSE_MAP);

		}
		if(frame_counter==0)
		{
			frame2_gray = frame1_gray_crop;
		}
		else
			frame2_gray = frame1_gray_aligned;
		//        split(frame, channels);
		//        ch1 = channels[0];
		//        ch2 = channels[1];
		//        ch3 = channels[2];
		//
		//        imshow("ch1", ch1);
		//        imshow("ch2", ch2);
		//        imshow("ch3", ch3);


		if(frame_counter>0)
		{
			IplImage copy = frame1_alligned;
			frame1_alligned_IPL = &copy;

			//Get HSV Image
			imHSV=convertImageRGBtoHSV(frame1_alligned_IPL);
			frame1_alligned_HSV = cv::cvarrToMat(imHSV);

			split(frame1_alligned_HSV, channels);
			H = channels[0];
			S = channels[1];
			V = channels[2];

			//Taking AIO for each channel
			//int ROI_size = 80;
			//Loop for Each LED
			int LED_status[10];

			for(int i=0;i<LED_no;i++)
			{
				Rect region_of_interest = Rect(LED_x[i]-abs(ROI_size/2),LED_y[i]-abs(ROI_size/2), ROI_size, ROI_size);

				Mat H_roi = H(region_of_interest);
				Mat S_roi = S(region_of_interest);
				Mat V_roi = V(region_of_interest);

				int H_roi_array[36000]={0},S_roi_array[36000]={0},V_roi_array[36000]={0};  //Change this based on size of ROI

				int roi_pix_count = 0;
				for(int j=0;j<(ROI_size);j++)
				{
					for(int k=0;k<(ROI_size);k++)
					{
						H_roi_array[roi_pix_count] = H_roi.at<uchar>(j,k);
						S_roi_array[roi_pix_count] = S_roi.at<uchar>(j,k);
						V_roi_array[roi_pix_count] = V_roi.at<uchar>(j,k);

						roi_pix_count++;
					}
				}

				//Sort the V ROI array
				std::vector<int> V_roi_array_vect(V_roi_array, V_roi_array + sizeof V_roi_array / sizeof V_roi_array[0]);

				std::vector<int> V_Roi_sort_ind = Sort_ROI(V_roi_array_vect);

				//Take top 10% values of the sorted V ROI and use those co ordinates
				std::vector<int> Roi_sort_ind = V_Roi_sort_ind;

				//Find mean values of chosen pixels in each channel.
				int mean_loop_count = 0,mean_H=0,mean_S=0,mean_V=0;
				int perc_10 = ROI_size*ROI_size/10;
				for(int j=36000-1;j>(36000-1-perc_10);j--)  //Change the j values based on ROI size
				{
					mean_H = mean_H + H_roi_array[Roi_sort_ind[j]];
					mean_S = mean_S + S_roi_array[Roi_sort_ind[j]];
					mean_V = mean_V + V_roi_array[Roi_sort_ind[j]];
					mean_loop_count++;
				}
				mean_H = mean_H/mean_loop_count;
				mean_S = mean_S/mean_loop_count;
				mean_V = mean_V/mean_loop_count;

				//Decision on LED on/off and LED color
				int S_thresh = 100, V_thresh = 40;

				//imshow("test", V_roi);
				//cvWaitKey(100);

				if(mean_S>S_thresh && mean_V>V_thresh)
					LED_status[i]=mean_H;
				else
					LED_status[i]=0;

				LED_map[i][frame_counter]=LED_status[i];

			}

		}

		//Filter the LED status
		if(frame_counter>3)
		{
			for(int m=0;m<LED_no;m++)
			{
				int win_array[3]={LED_map[m][frame_counter],LED_map[m][frame_counter-1],LED_map[m][frame_counter-2]};//,LED_map[m][frame_counter-3],LED_map[m][frame_counter-4]};
				sort(win_array, win_array + 3);
				LED_map_filtered[m][frame_counter] = win_array[1];

				//Display Function to show LED status

				std::ostringstream LED_status;
				if(win_array[1]==0)
					LED_status<<"LED_Off";
				else
					LED_status<<win_array[1];

				putText(frame1_alligned, LED_status.str(), cvPoint(LED_x[m],LED_y[m]+40),
					FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(250,250,0), 1, CV_AA);
			}

			//imshow("frame1",frame1_alligned);
			//cvWaitKey(1); // waits to display frame
		}
	}

	//Test -- Savi LED Map in text file
	for(int i=0;i<10;i++)
	{
		for(int j=0;j<62;j++)
		{
			ofstream mapFile;
			int pos1 = filename.find_last_of("/");
			string mapFileName = filename.substr(pos1+1,filename.size()-pos1);
			int pos = mapFileName.find_first_of(".");
			mapFileName.replace(pos,8,"_map.txt");
			mapFile.open(mapFileName.c_str(),std::ios_base::app | std::ofstream::binary);
			mapFile<<LED_map_filtered[i][j]<<" ";
			mapFile.close();
		}
	}
	//Get the final LED status
	int Z_count=0,NZ_count=0,LED_status_final[10]={0},LED_hue[10]={0};
	int mean_H=0;
	for(int m=0;m<LED_no;m++)
	{
		Z_count=0;NZ_count=0;
		for(int n=4;n<61;n++)
		{
			if(LED_map_filtered[m][n]==-0)
			{
				Z_count++;
			}
			else
			{
				NZ_count++;
				mean_H=mean_H+LED_map_filtered[m][n];
			}
		}
		mean_H=mean_H/(NZ_count+0.001);
		LED_hue[m]=mean_H;
		if(Z_count>50)
			LED_status_final[m]=0;
		else if (NZ_count>50)
			LED_status_final[m]=1;
		else
			LED_status_final[m]=2;
	}

	if(matching==0)
	{
		//Save LED Status in a text file
		ofstream LED_status_txt;
		int loc = (int)filename.find_last_of(".");
		string status_filename = filename.substr(0,loc) + "_status.txt";
		LED_status_txt.open (status_filename);
		LED_status_txt<<"NumberOfLED : "<<LED_no<<endl;
		LED_status_txt<<"LEDStatus : ";
		for(int i=0;i<10;i++)
		{
			LED_status_txt<<LED_status_final[i]<<",";
		}
		LED_status_txt<<endl;
		LED_status_txt<<"LEDHue : ";
		for(int i=0;i<10;i++)
		{
			LED_status_txt<<LED_hue[i]<<",";
		}
		LED_status_txt.close();
	}
	else
	{
		//Sample Code for MAtching File:

		DIR *dir;
		struct dirent *ent;
		int found_txt=0;
		if ((dir = opendir ("Matching_set")) != NULL) {
			/* print all the files and directories within directory */
			while ((ent = readdir (dir)) != NULL) {
				//printf ("%s\n", ent->d_name);
				string line1=ent->d_name;
				found_txt=(int)line1.find(".txt");
				int MatchFlag = 1;
				if(found_txt>1)
				{
					cout << line1 << '\n';
					string line2 = "Matching_set/" + line1;
					ifstream Matching_set_file(line2);
					int Line_No=0;
					int LED_status_file[10]={0};
					int LED_hue_file[10]={0};
					if (Matching_set_file.is_open())
					{
						while ( getline (Matching_set_file,line2) )
						{
							cout << line2 << '\n';
							Line_No++;
							int colon_loc = (int)line2.find(":");
							if(Line_No==1)
							{
								int LED_NO_file = stoi(line2.substr(colon_loc+1));
								if(LED_NO_file!=LED_no)
								{
									MatchFlag=0;
									break;
								}
							}
							if(Line_No==2)
							{
								for(int i=0;i<10;i++)
								{
									LED_status_file[i]=stoi(line2.substr(colon_loc+1,colon_loc+2));
									if(i==0)
										colon_loc=colon_loc+3;
									else
										colon_loc=colon_loc+2;
									if(LED_status_file[i]!=LED_status_final[i])
									{
										MatchFlag=0;
										break;
									}

								}
							}
							if(Line_No==3)
							{
								int found=0;
								for(int i=0;i<10;i++)
								{
									found=(int)line2.find_first_of(",",found+1);
									LED_hue_file[i]=stoi(line2.substr(colon_loc+1,found));
									colon_loc=found;
									if(abs(LED_hue_file[i]-LED_hue[i])>20)
									{
										MatchFlag=0;
										break;
									}

								}
								//cout<<"MATCH"<<endl;
							}
						}
						if(MatchFlag==0)
							cout<<"NO MATCH"<<endl;
						else
							cout<<"MATCH"<<endl;

						Matching_set_file.close();
					}

				}
			}
			closedir (dir);
		} else {
			/* could not open directory */
			perror ("");
			//return EXIT_FAILURE;
		}

	}

	//Releaseing Mat structures
	frame1.release();
	frame1_alligned.release();
	frame1_gray.release();
	frame2_gray.release();
	frame1_gray_aligned.release();
	frame1_gray_crop.release();
	cvReleaseImage(&imHSV);
	//cvReleaseImage(&frame1_alligned_IPL);
	//    B.release();
	//    G.release();
	//    R.release();
	return 0;
}
