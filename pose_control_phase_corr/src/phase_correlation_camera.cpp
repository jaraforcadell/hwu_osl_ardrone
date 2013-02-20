#include "phase_correlation_camera.h"



//--------CALLBACK CONTAINING THE IMAGE----------------------

void PhaseCorrelation::controller(const sensor_msgs::ImageConstPtr& msg) {

	//Transform the image to opencv format
   cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	
	Mat image = cv_ptr->image; 
	
	imshow("image", image);
	waitKey(1);

	//Transform image to grayscale
	cvtColor(image, image2_in, CV_RGB2GRAY); //Image2 is the newest image, the one we get from this callback

	//image1 is the image2 from the last callback (we use the one that already has the dft applied. 
	Mat image1_dft = last_image_dft;

	//Image2, preparation and dft
	 Mat padded2;                            //expand input image to optimal size
	 int m2 = getOptimalDFTSize( image2_in.rows );
	 int n2 = getOptimalDFTSize( image2_in.cols ); // on the border add zero values
	 copyMakeBorder(image2_in, padded2, 0, m2 - image2_in.rows, 0, n2 - image2_in.cols, BORDER_CONSTANT, Scalar::all(0));

	 Mat planes2[] = {Mat_<float>(padded2), Mat::zeros(padded2.size(), CV_32F)};
	 Mat image2,image2_dft;
	 merge(planes2, 2, image2);         // Add to the expanded another plane with zeros

	 dft(image2, image2_dft); 	


	//Image2 will be image1 on next iteration
	last_image_dft=image2_dft;

	if( !first){
	 	//obtain the cross power spectrum  c(u,v)=F(u,v)·G*(u,v)/abs(F(u,v)·G*(u,v))
		Mat divident, divisor, cross_power_spec, trans_mat;
		mulSpectrums(image1_dft, image2_dft, divident, 0, true); 	
																		//=F(u,v)·G*(u,v) --> multiply the result of a dft //divident-> where it stores the result.
																		// flags=0. conj=true, because i want the image2 to be the complex conjugate.
		divisor=abs(divident);

		divide(divident, divisor, cross_power_spec, 1);	

		//dft of the cross_power_spec
	 	dft(cross_power_spec, trans_mat, DFT_INVERSE);

		//Normalize the trans_mat so that all the values are between 0 and 1
		normalize(trans_mat, trans_mat, NORM_INF);

		//Split trans_mat in it's real and imaginary parts
		vector<Mat> trans_mat_vector;
		split(trans_mat, trans_mat_vector);
		Mat trans_mat_real = trans_mat_vector.at(0); 
		Mat trans_mat_im = trans_mat_vector.at(1); 

		imshow("trans_mat_real", trans_mat_real);
		waitKey(1);

		//Look for maximum value and it's location on the trans_mat_real matrix
		double* max_value;
		Point* max_location;

		double max_val;
		Point max_loc; 
		max_value = &max_val;
		max_location = &max_loc;
	
		minMaxLoc(trans_mat_real, NULL, max_value, NULL, max_location);

		ROS_INFO_STREAM("max_value: " << max_val << "  -  " << "max_location: " << max_loc);			

		int pixel_x, pixel_y;

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


		//Add the new displacement to the accumulated
		pixels_x = pixels_x + pixel_x;
		pixels_y = pixels_y + pixel_y;
		ROS_INFO_STREAM("pixels_x: " << pixels_x << "  -  " << "pixel_y: " << pixels_y);		

		//------ transform pixels to mm ---------
 
		//To get the focal lenght:
		if (first){	

			Mat cameraMatrix(3, 3, CV_32F);
			cameraMatrix.at<float>(0,0)= 672.03175; //Values from the camera matrix are from the visp calibration
			cameraMatrix.at<float>(0,1) = 0.00000;
			cameraMatrix.at<float>(0,2) = 309.39349;
			cameraMatrix.at<float>(1,0) = 0.00000;
			cameraMatrix.at<float>(1,1) = 673.05595;
			cameraMatrix.at<float>(1,2) = 166.52006; 
			cameraMatrix.at<float>(2,0) = 0.00000;
			cameraMatrix.at<float>(2,1) = 0.00000;
			cameraMatrix.at<float>(2,2) = 1.00000;

			double apertureWidth, apertureHeight, fovx, fovy, focalLength, aspectRatio;
			Point2d principalPoint;
 
 			calibrationMatrixValues(cameraMatrix, image.size(), apertureWidth, apertureHeight, fovx, fovy, focalLength,  principalPoint, aspectRatio);
	
			ROS_INFO_STREAM("focalLength: " << focalLength);
			fl = focalLength;

			first=false;
		}
		
		float mov_x = pixel_x/fl*h;
		float mov_y = pixel_y/fl*h;
		mov_x_acc =	mov_x_acc + mov_x;
		mov_y_acc = mov_y_acc + mov_y;

		ROS_INFO_STREAM("mov_x:  " << mov_x << " - mov_y:  " << mov_y);
		ROS_INFO_STREAM("mov_x_acc:  " << mov_x_acc << " - mov_y_acc:  " << mov_y_acc);

	}

}


//-----Navdata Callback---------------------------------------

void PhaseCorrelation::navdata(const ardrone_autonomy::Navdata::ConstPtr& msg){

//	h = msg -> altd; //This can only be used when the quadcopter is flying. For hand held trials: 
	h = 1000;

}

//---------------MAIN-----------------------------------------

int main(int argc, char** argv){

	ros::init(argc, argv, "phase_corr_camera");

	PhaseCorrelation ic;
	
	ros::spin();

	return 0;
}



//-------CONSTRUCTOR-----------
PhaseCorrelation::PhaseCorrelation() : it_(nh_)  {
	image_sub_ = it_.subscribe("ardrone/bottom/image_raw", 1, &PhaseCorrelation::controller, this);
	navdata_sub_ = nh_.subscribe("ardrone/navdata", 1000, &PhaseCorrelation::navdata, this);
	first = true;
	pixels_x = 0;
	pixels_y = 0;
	mov_x_acc = 0;
	mov_y_acc = 0;
}
