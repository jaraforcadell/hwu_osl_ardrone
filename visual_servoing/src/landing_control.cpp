#include "landing_control.h"

//--- Navdata Callback -------------------------------------------------------
void land::NavdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg){
	drone_state = msg -> state;
	battery = msg -> batteryPercent;
	altd = msg -> altd;

	if (drone_state == 0){
		drone = "Unknown";
	} else if (drone_state == 1){
		drone = "Initied";
	} else if (drone_state == 2){
		drone = "Landed";
	} else if (drone_state == 3){
		drone = "Flying";
	} else if (drone_state == 4){
		drone = "Hovering";
	} else if (drone_state == 5){
		drone = "Test";
	} else if (drone_state == 6){
		drone = "Taking off";
	} else if (drone_state == 7){
		drone = "Flying";
	} else if (drone_state == 8){
		drone = "Landing";
	}

	//Check the battery
	if (battery <= 6){
		battery_low = true;
		status = LANDING;
		info = "BATTERY LOW";
	}

	if (status == TAKEOFF){
		status_info = "TAKE OFF";
  	info = "----";
		if (drone_state != 4){
			//Take off
			msg_takeoff.priority = false;
			pub_takeoff.publish(msg_takeoff);
		}

		if (drone_state == 4)
			status = SEARCHING;

	} else if (status == SEARCHING){
		status_info = "SEARCHING";
		if (!init_detector){
			//Wait for the target detection server to start
			detection.waitForServer();
			info = "Target searcher initialised";

			//Send a goal to the action
			visual_servoing::Tag_poseGoal detecting_goal;
			detection.sendGoal(detecting_goal, boost::bind(&land::detection_doneCb, this, _1), detect_client::SimpleActiveCallback(), boost::bind(&land::detection_feedbackCb, this, _1));
			init_detector = true;
		}

	} else if (status == TRACKING){
		status_info = "TRACKING";
		if (!init_tracker){
			//Wait for the tracking server to start
			tracking.waitForServer();
			info = "tracking initialised";

			//Send a goal to the action
			visual_servoing::trackingGoal tracking_goal;
			tracking.sendGoal(tracking_goal, boost::bind(&land::tracking_doneCb, this, _1), track_client::SimpleActiveCallback(), boost::bind(&land::tracking_feedbackCb, this, _1));
			init_tracker = true;
		}

		if (!tracking_working){
	  	info = "Target lost!";
			status = LOST;
		}

	} else if (status == LOST){
		status_info = "LOST";
		//Wait a while, if found go to TRACKING status again, if not go to the timer lost callback and there to the SEARCHING status
		if (!tracking_working){
			timer_lost.start();
		} else{
			timer_lost.stop();
			status = TRACKING;
		}
	
	} else if (status == LANDING){
		status_info = "LANDING";
		//Land
		//Move the drone forwards to land over the target
		msg_move.linear.x = 0.3;
		msg_move.linear.y = 0;
		msg_move.angular.z = 0;
		msg_move.priority = false;
		pub_move.publish(msg_move);

		if (ai){
			if (battery_low == true){
				msg_land.priority = true;
			} else{
				msg_land.priority = false;
			}
			pub_land.publish(msg_land);

			ai = false;
		}
	}

	//Call the printing function to show the control in the screen
	print();
}


// ------ Detection feedback callback -------------------------------------------------------
void land::detection_feedbackCb(const visual_servoing::Tag_poseFeedbackConstPtr& feedback){
	found = feedback->found;
	foundSure = feedback->foundSure;
	tag_x = feedback->tag_x;
	tag_y = feedback->tag_y;

	//Search the target between 800mm and 1300mm (the detection works well in this altitudes)
	if (altd > 1300){
		msg_move.linear.z = -0.08;
		msg_move.linear.x = 0;
		msg_move.linear.y = 0;
		msg_move.angular.z = 0;
	} else if (altd < 800){
		msg_move.linear.z = 0.08;
		msg_move.linear.x = 0;
		msg_move.linear.y = 0;
		msg_move.angular.z = 0;
	} else{
		if (found == false){
			info = "Searching the target";
			msg_move.linear.x = 0.015;
			msg_move.linear.y = 0;
			msg_move.linear.z = 0;
			msg_move.angular.z = 0;
		} else{
			if(foundSure == false){
				//When the target is found, wait for the verification
				info = "Target found. Verifying";
				msg_move.linear.x = 0;
				msg_move.linear.y = 0;
				msg_move.linear.z = 0;
				msg_move.angular.z = 0;
			} else{
				info = "Target verified";
			}
		}
	}
	msg_move.priority = false;
	pub_move.publish(msg_move);
}


// ------ Detection done callback --------------------------------------------
void land::detection_doneCb(const actionlib::SimpleClientGoalState& state){
	//Stop the drone
	msg_move.linear.x = 0;
	msg_move.linear.y = 0;
	msg_move.linear.z = 0;
	msg_move.angular.z = 0;
	msg_move.priority = false;
	pub_move.publish(msg_move);

	//Change to tracking status
	status = TRACKING;
}


// ------- Tracking feedback callback ------------------------------------------------------
void land::tracking_feedbackCb(const visual_servoing::trackingFeedbackConstPtr& feedback){
	tracking_working = feedback->working;
	vel_x = feedback->vel_x;
	vel_y = feedback->vel_y;
	tag_x = feedback->centre_x;
	tag_y = feedback->centre_y;

	if (tracking_working){
		//Center the target and decend
		info = "Tracking working. Approaching";
		msg_move.linear.x = vel_x;
		msg_move.linear.y = vel_y;

		//If the target is in the centre (22cm square) descend
		if (abs(tag_x) > 22 || abs(tag_y) > 22){
			msg_move.linear.z = 0;
		} else{
			msg_move.linear.z = -0.1;
		}

		msg_move.angular.z = 0;
		msg_move.priority = false;
		pub_move.publish(msg_move);

	} else{
		info = "TRACKING NOT WORKING!";
	}
}


// ----- Tracking done callback ---------------------------------------------
void land::tracking_doneCb(const actionlib::SimpleClientGoalState& state){
	if (strcmp(state.toString().c_str(),"SUCCEEDED") == 0){
		//Move the drone forwards to land over the target
		msg_move.linear.x = 0.3;
		msg_move.linear.y = 0;
		msg_move.angular.z = 0;
		msg_move.priority = false;
		pub_move.publish(msg_move);

		for(int i = 0; i < 5; i++ ){}
		//Change to landing status
		status = LANDING;
		info = "Mission finished?";

	} else{
		//The state is not SUCCEEDED, that means that the goal has been preempted, so search the target again
		status = SEARCHING;
	}
}


// ------ Timer lost ------------------------------------------
void land::LostCallback(const ros::TimerEvent& event){
	timer_lost.stop();
	tracking_working = false;
	init_tracker = false;
	init_detector = false;
	tracking.cancelGoal();
}


// ------ Print function ------------------------------------------------------------
void land::print(){
		float sec = 0.05;

		ROS_INFO_STREAM_THROTTLE(sec, "--------------LANDING CONTROL--------------");
		ROS_INFO_STREAM_THROTTLE(sec, "Code status:	" << status_info);
		ROS_INFO_STREAM_THROTTLE(sec, "Battery:	" << battery << "%");
		ROS_INFO_STREAM_THROTTLE(sec, "Drone state:	" << drone);
		ROS_INFO_STREAM_THROTTLE(sec, "Altitude:	" << altd);
		ROS_INFO_STREAM_THROTTLE(sec, "INFO: " << info);
		ROS_INFO_STREAM_THROTTLE(sec, "Centre of the target:	" << tag_x << ", " << tag_y);
		ROS_INFO_STREAM_THROTTLE(sec, "------------------CONTROL------------------");
		ROS_INFO_STREAM_THROTTLE(sec, "Velocities");
		ROS_INFO_STREAM_THROTTLE(sec, "----------");
		ROS_INFO_STREAM_THROTTLE(sec, "Velocity x:	" << msg_move.linear.x);
		ROS_INFO_STREAM_THROTTLE(sec, "Velocity y:	" << msg_move.linear.y);
		ROS_INFO_STREAM_THROTTLE(sec, "Velocity z:	" << msg_move.linear.z);
		ROS_INFO_STREAM_THROTTLE(sec, "Yaw:		" << msg_move.angular.z);
		ROS_INFO_STREAM_THROTTLE(sec, "-------------------------------------------" << std::endl);

		ROS_INFO_STREAM_THROTTLE(sec, std::endl);
}


// -------- Main ---------------------------------------------------------
int main(int argc, char **argv){
	ros::init(argc, argv, "landing_controller");

	land landing_controller;

	ros::spin();
	return 0;
}


// -------- Constructor -------------------------------------------------------------------
land::land():
//true causes the client to spin its own thread
detection("target_detection", true),
tracking("tracking", true)
{
	//Set to hd camera
	client = nh_.serviceClient<ardrone_autonomy::CamSelect>("ardrone/setcamchannel");
	srv.request.channel = 0;
	client.call(srv);

	sub_ardrone = nh_.subscribe("ardrone/navdata", 1000, &land::NavdataCallback,this);
	pub_takeoff = nh_.advertise<ardrone_msgs::Priority>("spooler/takeoff", 1000);
	pub_move = nh_.advertise<ardrone_msgs::Vel>("spooler/cmd_vel", 1000);
	pub_land = nh_.advertise<ardrone_msgs::Priority>("spooler/land",1000);

	//Initialisation of variables
	//Set all velocities to 0
	msg_move.linear.x=0;
	msg_move.linear.y=0;
	msg_move.linear.z=0;
	msg_move.angular.z=0;

	timer_lost = nh_.createTimer(ros::Duration(3), &land::LostCallback, this);
	timer_lost.stop();

	status = TAKEOFF;
	tracking_working = false;
	init_takeoff = false;
	init_tracker = false;
	init_detector = false;
	battery_low = false;
	ai = true;

	altd = 0;
	tag_x = 0;
	tag_y = 0;
	drone_state = 0;

	info = "Landing control node started";
	print();
}
