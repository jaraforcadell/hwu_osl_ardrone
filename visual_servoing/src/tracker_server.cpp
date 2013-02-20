#include "tracker_server.h"

// ----- Goal callback ------------------------------------------------------------
//to start the tracking when is required by the master
void Tracking::goalCB(){
	start = true;

	Server_.acceptNewGoal();
	//Make sure the goal hasn't been cancelled between it was found and we accept it, as between his time period preemt callback is not triggered
	if(Server_.isPreemptRequested()){
		preemptCB();
	}
}

// ------ Preempt callback -------------------------------------
//to cancel the goal when required by the master
void Tracking::preemptCB(){
	Server_.setPreempted();
	start = false;
	first = true;
	working = false;
	taskPtr->kill();
	delete taskPtr;
}


// ------ Navdata callback ------------------------------------------------------
//to get the altitude and the battery percent)
void Tracking::NavdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg){
	altd = msg -> altd;
	battery = msg -> batteryPercent;
}


// ------ Camera info callback --------------------------------------------------
//to get the camera parameters
void Tracking::camerainfo(const sensor_msgs::CameraInfo& msg){
	sensor_msgs::CameraInfo cam_info=msg;
	cam = visp_bridge::toVispCameraParameters(cam_info);
}


// ------ Controller callback ---------------------------------------------------- 
//contains the images
void Tracking::controller(const sensor_msgs::ImageConstPtr& msg){
	if (start){
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		frame = cv_ptr->image;

		//Convert image to gray and to VISP vpImage class
		cvtColor(frame, frame, CV_RGB2GRAY);
		vpImageConvert::convert(frame, I);

		//If first is true, do the initialisation
		if(first){
			//Display initialisation
			d.init(I, 0, 0, "Tracking");

			vpDisplay::display(I);
			vpDisplay::flush(I);

			//Initialise the tracking
			dot.setGrayLevelPrecision(0.9);
			//To allow the tracking of a non ellipoid shape
			dot.setEllipsoidShapePrecision(0);
			//Get the points of the centre of the target
			nh_.getParam("/target/x", target_x);
			nh_.getParam("/target/y", target_y);
			ip.set_i(target_y);
			ip.set_j(target_x);

	 		dot.initTracking(I, ip);
			vpDisplay::flush(I);

			//Initialize visual servoing task
			cog = dot.getCog();
			tag.setWorldCoordinates(I.getHeight()/2, I.getWidth()/2, 1);
			vpFeatureBuilder::create(sd,tag);

			taskPtr = new vpServo();
			taskPtr->setServo(vpServo::EYEINHAND_CAMERA);
			taskPtr->setInteractionMatrixType(vpServo::DESIRED);
			taskPtr->addFeature(s,sd);

			first = false;
		}

		//Set the lambda of the visual servoing depending on the battery
		if (altd >= 700){
			if (battery > 70){
				lambda = 0.33;
			} else if(battery > 50){
				lambda = 0.35;
			} else{
				lambda = 0.32;
			}
		} else{
			if (battery > 70){
				lambda = 0.25;
			} else if(battery > 50){
				lambda = 0.22;
			} else{
				lambda = 0.18;
			}
		}
		taskPtr->setLambda(lambda);

		vpDisplay::display(I);

		//Try to do the tracking of the dot, if it fails, set working to false
		try{
			//Track the dot
			dot.track(I);
			dot.display(I, vpColor::cyan, 3);
			vpFeatureBuilder::create(s, cam, dot);
			v = taskPtr->computeControlLaw();

			working = true;
		}
		catch (...){
			working = false;
			SendPose();
		}

		//Get the centre of gravity of the dot (which is the centre of the target) if it is tracked
		if (working){
			cog = dot.getCog();
			centre_x = -(cog.get_i() - frame.size().height/2);
			centre_y = cog.get_j() - frame.size().width/2;

			//Transformation the target centre into cm
			k = (-0.03632) * altd + (32.802867);
			centre_x = centre_x / k;
			centre_y = centre_y / k;
		}

		//Set the velocities to send to the drone calculated with the visual servoing task
		vel_x = -v[1];
		vel_y = -v[0];

		//Display the centre of the dot and the goal position of it in the image
		vpDisplay::displayCross(I, vpImagePoint(I.getHeight()/2, I.getWidth()/2), 20, vpColor::red, 2);

		vpDisplay::flush(I);

		//To publish the video
		vpDisplay::getImage(I,Ivideo);
		vpImageConvert::convert(Ivideo, video);
		cv_bridge::CvImagePtr cv_ptr_out(new cv_bridge::CvImage);
		cv_ptr_out->image = video;
		cv_ptr_out->encoding = "bgr8";
		sensor_msgs::Image message;
		cv_ptr_out->toImageMsg(message);
		image_pub_.publish(message);

		//If working and centred, don't send
		if (working && altd < 275 && abs(centre_x) < 30 && abs(centre_y) < 30){
		//If its difficult to land: remove the condition to be centred
			if (sure){
				//Move forwards to land over the target
				vel_x = 0.3;
				SendPose();

				//Set the action state to succeeded
				Server_.setSucceeded(result_);
				start = false;
			}
			sure = true;
		}

		//Call the function to send the feedback to the master
		SendPose();
	}
}


// ------ SendPose callback ---------------------------
//to send the feedback to the master
void Tracking::SendPose(){
	feedback_.working = working;
	feedback_.vel_x = vel_x;
	feedback_.vel_y = vel_y;
	feedback_.centre_x = centre_x;
	feedback_.centre_y = centre_y;
	Server_.publishFeedback(feedback_);
}


// -------- Main ----------------------------------------
int main(int argc, char** argv){
	ros::init(argc, argv, "tracker_pub");

	Tracking ic;

	ros::spin();
	return 0;
}

//---------- Destructor --------------------------------------

Tracking::~Tracking(){
	taskPtr->kill();
	delete taskPtr;
}


// --------- Constructor ---------------------------------------------------------------------------
Tracking::Tracking():
it_(nh_),
Server_(nh_,"tracking", false)
{
	//Register the Goal and the Preept Callbacks
	Server_.registerGoalCallback(boost::bind(&Tracking::goalCB,this));
	Server_.registerPreemptCallback(boost::bind(&Tracking::preemptCB,this));

	sub_ardrone = nh_.subscribe("ardrone/navdata", 1000, &Tracking::NavdataCallback,this);
	image_sub_ = it_.subscribe("ardrone/front/image_rect_color", 1, &Tracking::controller, this);
	camera_info_sub_ =nh_.subscribe("ardrone/front/camera_info", 1, &Tracking::camerainfo, this);
	image_pub_ = it_.advertise("tracking/image", 1);

	//Start the server
	Server_.start();

	//Initialisation of variables
	start = false;
	first = true;
	working = true;
	sure = false;
}
