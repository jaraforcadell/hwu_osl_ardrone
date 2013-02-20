#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/Image.h"
#include <ardrone_autonomy/Navdata.h>
#include <conversions/camera.h>

#include <actionlib/server/simple_action_server.h>
#include <visual_servoing/trackingAction.h>

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpDot2.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpConfig.h>
#include <visp/vpVideoWriter.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

typedef actionlib::SimpleActionServer<visual_servoing::trackingAction> TrackServer;

class Tracking{

public:
	Tracking();
	~Tracking();

	//Functions
	void goalCB();
	void preemptCB();
	void controller(const sensor_msgs::ImageConstPtr& msg);
	void NavdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);
	void camerainfo(const sensor_msgs::CameraInfo& msg);
	void SendPose();

protected:
	ros::NodeHandle nh_;

	//Subscribers and publishers
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber sub_ardrone;
	ros::Subscriber camera_info_sub_;
	image_transport::Publisher image_pub_;

	//Create the server and messages that are used to published feedback/result
	TrackServer Server_;
	visual_servoing::trackingFeedback feedback_;
	visual_servoing::trackingResult result_;

	//Node variables
	//Visual servoing variables
	vpImage<unsigned char> I;
	vpDisplayOpenCV d;
	vpServo* taskPtr;
	vpDot2 dot;
	vpPoint tag;
	vpImagePoint ip, cog;
	vpCameraParameters cam;
	vpFeaturePoint s, sd;
	vpColVector v;
	vpImage<vpRGBa> Ivideo;
	//Other node variables
	Mat frame, video;
	bool start, first, working, sure;
	double target_x, target_y;
	double lambda;
	float vel_x, vel_y, k;
	int centre_x, centre_y, altd;
	float battery;
};
