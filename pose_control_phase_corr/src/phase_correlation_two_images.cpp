#include <ros/ros.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

int main(int argc, char ** argv){

	Mat image1_in = imread("im_1.jpg", CV_LOAD_IMAGE_GRAYSCALE);  

	Mat image2_in = imread("im_2.jpg", CV_LOAD_IMAGE_GRAYSCALE);   

	imshow("image1_in", image1_in);
	waitKey(100);
	imshow("image2_in", image2_in); 
	waitKey(100);   
		
	//Image1, preparation and dft
   Mat padded1;                            //expand input image to optimal size
   int m1 = getOptimalDFTSize( image1_in.rows );
   int n1 = getOptimalDFTSize( image1_in.cols ); // on the border add zero values
   copyMakeBorder(image1_in, padded1, 0, m1 - image1_in.rows, 0, n1 - image1_in.cols, BORDER_CONSTANT, Scalar::all(0));

   Mat planes1[] = {Mat_<float>(padded1), Mat::zeros(padded1.size(), CV_32F)};
   Mat image1;
   merge(planes1, 2, image1);         // Add to the expanded another plane with zeros. We obtain a double channel array. 
													//  planes is the source array, image1 the destination array, 2 num. of source arrays
   dft(image1, image1); 

	//Image2, preparation and dft
    Mat padded2;                            //expand input image to optimal size
    int m2 = getOptimalDFTSize( image2_in.rows );
    int n2 = getOptimalDFTSize( image2_in.cols ); // on the border add zero values
    copyMakeBorder(image2_in, padded2, 0, m2 - image2_in.rows, 0, n2 - image2_in.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes2[] = {Mat_<float>(padded2), Mat::zeros(padded2.size(), CV_32F)};
    Mat image2;
    merge(planes2, 2, image2);         // Add to the expanded another plane with zeros

    dft(image2, image2); 


 	//obtain the cross power spectrum  c(u,v)=F(u,v)·G*(u,v)/abs(F(u,v)·G*(u,v))
	Mat divident, divisor, cross_power_spec, trans_mat;
	mulSpectrums(image1, image2, divident, 0, true); 	// divident = F(u,v)·G*(u,v) 
																		// conj=true, so thesecond argument will be the complex conjugate of image 2
	divisor=abs(divident);

	divide(divident, divisor, cross_power_spec, 1);	

	//inverse dft of the cross_power_spec
 	dft(cross_power_spec, trans_mat, DFT_INVERSE); 

	//Normalize the trans_mat so that all the values are between 0 and 1
	normalize(trans_mat, trans_mat, NORM_INF);

	//Split trans_mat in it's real and imaginary parts
	vector<Mat> trans_mat_vector;
	split(trans_mat, trans_mat_vector);
	Mat trans_mat_real = trans_mat_vector.at(0); 
	Mat trans_mat_im = trans_mat_vector.at(1); 

	imshow("trans_real", trans_mat_real);
	waitKey(1000);

	//Look for the maximum value and it's location on the trans_mat_real matrix
	double* max_value;
	Point* max_location;
	double max_val;
	Point max_loc; 
	max_value = &max_val;
	max_location = &max_loc;

	double* min_value;
	Point* min_location;
	double min_val;
	Point min_loc; 
	min_value = &min_val;
	min_location = &min_loc;

	minMaxLoc(trans_mat_real, min_value, max_value, min_location, max_location);
	ROS_INFO_STREAM("Image dimensions in pixels: width: " << image1.cols  << " , " << "height: " << image1.rows);
 	ROS_INFO_STREAM("max_value: " << max_val << "   -    max_location: " << max_loc);
	
	int pixel_x, pixel_y;

//Depending on the quadrant the point is located:

	if(max_loc.x < (image2.cols/2) && max_loc.y < (image2.rows/2)){  // top-left quadrant
		ROS_INFO_STREAM(" top - left quadrant");
		pixel_x = max_loc.x;
		pixel_y = - max_loc.y;
	}
	if(max_loc.x > (image2.cols/2) && max_loc.y > (image2.rows/2)){  // lower-right quadrant
		ROS_INFO_STREAM(" lower - right quadrant");
		pixel_x = - image2.cols + max_loc.x;
		pixel_y = + image2.rows - max_loc.y;
	}
	if(max_loc.x > (image2.cols/2) && max_loc.y < (image2.rows/2)){  // top-right quadrant
		ROS_INFO_STREAM(" top - right quadrant");
		pixel_x = - image2.cols + max_loc.x;
		pixel_y = - max_loc.y;
	}
	if(max_loc.x < (image2.cols/2) && max_loc.y > (image2.rows/2)){  // lower-left quadrant  
		ROS_INFO_STREAM(" lower - left quadrant");
		pixel_x = max_loc.x;
		pixel_y = image2.rows - max_loc.y;
	}


 	ROS_INFO_STREAM("pixel_x " << pixel_x << "   -    pixel_y: " << pixel_y);

	waitKey(0); //waits for a key to close all the windows
}

