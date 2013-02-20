#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "sensor_msgs/Image.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ardrone_autonomy/Navdata.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class PhaseCorrelation
{

public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber navdata_sub_;

	PhaseCorrelation(); 
	void controller(const sensor_msgs::ImageConstPtr& msg);
	void navdata(const ardrone_autonomy::Navdata::ConstPtr& msg);

	bool first;
	int pixels_x, pixels_y;
	float mov_x, mov_y;
	float mov_x_acc, mov_y_acc;
	int h;
	double fl;

	Mat image2_in; 
	Mat last_image_dft,image1_dft;
};
