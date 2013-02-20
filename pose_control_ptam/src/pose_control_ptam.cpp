#include "pose_control_ptam.h"

//--------- Pose callback  -------------------------------------------------------------------------------------
//active every time it receives the pose from PTAM
void pose_controller::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

	if (state == LOST){//To stop the lost timer and stop the drone if the ardrone founds the map while turning
		info = "Ardrone has found the lost map - Stopping timer_lost"; 

		//Set all velocities to zero
		msg_move.linear.x = 0;
		msg_move.linear.y = 0;		
		msg_move.linear.z = 0;
		msg_move.angular.z = 0;
		msg_move.priority = false;
		pub_move.publish(msg_move);	

		//Stop the lost timer
		timer_lost.stop();           
	}

	//Since the pose from PTAM is received, the map is working, and the state is POSITION KEEPING
	map_works_ = true;
	current_state = POSITION_KEEPING;
	info = "Doing position keeping";
	time_stamp_last_msg_ = msg->header.stamp;


	//Start the scaling the map
	if (scale == INIT_SCALING){ 
		current_scale = "INIT_SCALING";
		//Get the value of "y" of the PTAM and the altitude from the altitude sensor
		z0 = msg->pose.pose.position.y;
		alt0 = alt;
		msg_move.linear.z = 0.3;
		msg_move.priority = false;
		pub_move.publish(msg_move);

		timer_scale.start();
		scale = MOVING;
	}
	else if (scale == MOVING){
		//Going up
		current_scale = "MOVING";
	}
	else if (scale == MOVED){
		current_scale = "MOVED";
		//Get the value of "y" of the PTAM and the altitude from the altitude sensor
		z1 = msg->pose.pose.position.y;
		alt1 = alt;

		scale = END_SCALING;
	}
	else if (scale == END_SCALING){
		current_scale = "END_SCALING";
		alt_diff = alt1 - alt0;
		alt_diff_ptam = z1 - z0;
		//Calculate scale (in meters) with the two difference of altitude
		sc = (alt_diff/alt_diff_ptam)/1000;
		msg_move.linear.z = -0.3;
		msg_move.priority = false;
		pub_move.publish(msg_move);

		timer_scale.start();
		scale = GO_DOWN;
	}
	else if (scale == GO_DOWN){
		//Going down to the initial position
		current_scale = "GO_DOWN";
	}
	//Scaling the map ended

	//Memorizes the initial position
	if (first == true && scale == SCALED){
		goal_x = -sc * msg->pose.pose.position.z;
		goal_y = sc * msg->pose.pose.position.x;
		goal_z = sc * msg->pose.pose.position.y;
		//Quaternion to Euler
		tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
		btMatrix3x3(q).getRPY(pitch, yaw, roll);
		goal_yaw = yaw;
		first = false;
	}

	//---------------------------------------------------------------------------------------

	//Position keeping controller
	margin = 0.1;//Margin of the goal position
	big_margin = 2 * margin;//Big margin (to increase the speed when is far from the goal)

	if (scale == SCALED && first == false){
		//Angle error calculation and correction:
		//Quaternion to Euler
		tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
		btMatrix3x3(q).getRPY(pitch, yaw, roll);
		error_yaw = yaw - goal_yaw;
		
		if (fabs(error_yaw) > margin){
			msg_move.angular.z = -error_yaw*0.9;
		}
		else {msg_move.angular.z = 0;}
		
 		if(!n.getParam("/gain", gain)){
			gain = 0.07;
			n.setParam("/gain", gain);
		}

		//X position error calculation and correction:
		x = -sc * msg->pose.pose.position.z;
		error_x = x - goal_x;
		if (fabs(error_x) > sc*margin && fabs(error_yaw) < margin){
			if (fabs(error_x) > sc*big_margin)
				{msg_move.linear.x = -error_x * gain * 2;}
			else
				{msg_move.linear.x = -error_x * gain;}
		}
		else {msg_move.linear.x = 0;}
	
		//Y position error calculation and correction:
		y = sc * msg->pose.pose.position.x;
		error_y = y - goal_y;
		if ( fabs(error_y) > sc*margin && fabs(error_yaw) < margin){
			if (fabs(error_y) > sc*big_margin)
				{msg_move.linear.y = -error_y * gain * 2;}
			else
				{msg_move.linear.y = -error_y * gain;}
		}
		else {msg_move.linear.y = 0;}
		
		//Altitude error calculation and correction:
		z = sc * msg->pose.pose.position.y;
		error_z = z - goal_z;
		ROS_INFO_STREAM("Error_z:" << error_z);
		if (fabs(error_z) > sc*margin){
			if (fabs(error_z) > sc*big_margin)
				{msg_move.linear.z = -error_z * gain * 2;}
			else
				{msg_move.linear.z = -error_z * gain;}
		}
		else {msg_move.linear.z = 0;}
	
		//Publish velocity messages:
		msg_move.priority = false;
		pub_move.publish(msg_move);
	}
}


// ------- Control Callback (active whenever the drone is running) -------------------------------------------------
//To control the states of the drone, to control the drone when the map from PTAM is not active and to get the altitude from the navdata
void pose_controller::controlCallback(const ardrone_autonomy::Navdata::ConstPtr& msg){
	//Reading the altitude and the drone_state
	alt = msg -> altd;
	drone_state = msg -> state;

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

	//To check if the ardrone is mapping
	//Wait one second to get the next map message, if not received in less than a second, set map_works_ as false
	if(map_works_){
		ros::Duration time_diff = ros::Time::now()-time_stamp_last_msg_;
		ros::Duration duration(1);
		if (time_diff > duration){
			info = "Map doesn't work. More than a second since the las map message";
			map_works_ = false;
		}
	}

	//---------------------------------------------------------------------------------------
	//Start the procedure to create a map
	if (state == TAKEOFF){
		current_state = "TAKEOFF";
		msg_takeoff.priority = false;
		pub_takeoff.publish(msg_takeoff);

		if(drone_state == 4){
			state = START_INIT;
		} 
	}
	if (state == START_INIT ){ // start map init
		current_state = "START_INIT";

		//Set a spacebar to take the features in PTAM node
		std_msgs::String msg_spacebar;
		msg_spacebar.data = "Space";
		pub_map.publish(msg_spacebar);

		//Go up
		msg_move.linear.x = 0;
		msg_move.linear.y = 0;		
		msg_move.linear.z = 0.5;
		msg_move.angular.z = 0;
		msg_move.priority = false;
		pub_move.publish(msg_move);

		//Timer will wait 0.5 seconds and then set state END_INIT
		timer_move.start();
		state = MOVING_TO_INIT; 
	}

	if(state == MOVING_TO_INIT){
		current_state = "MOVING_TO_INIT";
	}

	if (state == END_INIT){
		current_state = "END_INIT";

		//Set a spacebar to take the features in PTAM node
		std_msgs::String msg_spacebar;
		msg_spacebar.data = "Space";
		pub_map.publish(msg_spacebar);

		//Stop the drone
		msg_move.linear.x = 0;
		msg_move.linear.y = 0;
		msg_move.linear.z = 0;
		msg_move.angular.z = 0;
		msg_move.priority = false;
		pub_move.publish(msg_move);

		timer_wait.start();
	}

	if (state == RE_INIT){
		current_state = "RE_INIT";

		//Go down to the initial position to start the map again
		msg_move.linear.x = 0;
		msg_move.linear.y = 0;		
		msg_move.linear.z = -0.5;
		msg_move.angular.z = 0;
		msg_move.priority = false;
		pub_move.publish(msg_move);	

		timer_move.start();
	}

	if (state == POSITION_KEEPING && map_works_ == false){
		//If the map is lost
		state = LOST;
		timer_lost.start();
	}

	if(state==LOST){
		current_state = "LOST";
		info = "Ardrone is lost - Searching map";

		//Rotate to find the previous map
		msg_move.angular.z = (fabs(error_yaw)/(error_yaw))*-0.3;
		//Set all other velocities to zero
		msg_move.linear.x = 0;
		msg_move.linear.y = 0;
		msg_move.linear.z = 0;
		msg_move.priority = false;
		pub_move.publish(msg_move);
		}

	print();
}


// ------- Timer move -------------------------------------------------
void pose_controller::MovingCallback(const ros::TimerEvent& event) {
	timer_move.stop();

	if (state == RE_INIT){
		state = START_INIT;
	}
	else if (state == MOVING_TO_INIT){
		state = END_INIT;
	}
}	

// -------- Timer lost ------------------------------------------------
void pose_controller::LostCallback(const ros::TimerEvent& event){
	timer_lost.stop();	
	state = START_INIT;

	//Stop the rotation of the ardrone
	msg_move.linear.x = 0;
	msg_move.linear.y = 0;
	msg_move.linear.z = 0;
	msg_move.angular.z = 0;
	msg_move.priority = false;
	pub_move.publish(msg_move);	

	//And reset the map, so when it goes back to state START_INIT it will start a new map
	std_msgs::String msg_reset;
	msg_reset.data = "r";
	pub_map.publish(msg_reset);

	//Now it will start a new map 
	first = true;
	scale = INIT_SCALING;
}	

// --------- Timer wait -----------------------------------------------
void pose_controller::WaitCallback(const ros::TimerEvent& event) {
	timer_wait.stop();

	if (map_works_ == false){	
		state=RE_INIT;
	}
}

// --------- Timer scale -----------------------------------------------
void pose_controller::ScaleCallback(const ros::TimerEvent& event) {
	timer_scale.stop();

	msg_move.linear.x = 0;
	msg_move.linear.y = 0;
	msg_move.linear.z = 0;
	msg_move.angular.z = 0;
	msg_move.priority = false;
	pub_move.publish(msg_move);

	if (scale == MOVING){
	scale = MOVED;
	}
	else if (scale == GO_DOWN){
	scale = SCALED;
	current_scale = "SCALED";
	}
}


// ---------- Init callback ----------------------------------------------
//To start the map from a desired position. It's to command the drone to a desired position and start a map and do the position keeping in there
void pose_controller::initCallback(const ardrone_msgs::Priority::ConstPtr& msg){
	info = "Reset map";

	//Set all the initial values
	timer_move.stop();
	timer_lost.stop();
	timer_scale.stop();
	timer_wait.stop();
	timer_x.stop();
	timer_y.stop();

	std_msgs::String msg_reset;
	msg_reset.data = "r";
	pub_map.publish(msg_reset);

	map_works_ = false;  
	first = true;
	time_x = false;
	time_y = false;
	state = START_INIT;
	scale = INIT_SCALING;
}


// ----------- Waypoint callback ------------------------------------------------------
//This callback allows us to add one meter in the x or y goal position
void pose_controller::waypointCallback(const ardrone_msgs::Vel::ConstPtr& msg) {
	//move the goal position forwards
	if ( msg->linear.x > 0 && time_x == false){
		goal_x = goal_x + 1;
		info = "Goal moved 1m forwards";
		timer_x.start();
		time_x = true;
	}
	//move the goal position backwards
	if ( msg->linear.x < 0 && time_x == false){
		goal_x = goal_x - 1;
		info = "Goal moved 1m backwards";
		timer_x.start();
		time_x = true;
	}
	//move the goal position to the right
	if ( msg->linear.y > 0 && time_y == false){
		goal_y = goal_y + 1;
		info = "Goal moved 1m to the right";
		timer_y.start();
		time_y = true;
	}
	//move the goal position to the left
	if ( msg->linear.y < 0 && time_y == false){
		info = "Goal moved 1m to the left";
		goal_y = goal_y - 1;
		timer_y.start();
		time_y = true;
	}
}

//Timer "x" and timer "y" are made to accept no more than one waypoint every second
// ----- Timer "x" ----------------------------------------------
void pose_controller::xCallback(const ros::TimerEvent& event){
	time_x = false;
	timer_x.stop();
}

// ----- Timer "y" ----------------------------------------------
void pose_controller::yCallback(const ros::TimerEvent& event){
	time_y = false;
	timer_y.stop();
}


//------- Print function ----------------------------------------------
void pose_controller::print(){
	ROS_INFO_STREAM("--------------POSE CONTROL--------------");
/*	ROS_INFO_STREAM("	Position	Goal position");
	ROS_INFO_STREAM("x: " << x << "	" << goal_x);
	ROS_INFO_STREAM("y: " << y << "	" << goal_y);
	ROS_INFO_STREAM("z: " << z << "	" << goal_z << std::endl);
*/
	ROS_INFO_STREAM("Drone state: " << drone);
	ROS_INFO_STREAM("Code state: " << current_state);
	ROS_INFO_STREAM("Scale state: " << current_scale);
	ROS_INFO_STREAM("Scale: " << sc);

	ROS_INFO_STREAM("Info: " << info);

	ROS_INFO_STREAM("------------CONTROL COMMANDS------------");
	ROS_INFO_STREAM("	Error");
	ROS_INFO_STREAM("x:	" << error_x);
	ROS_INFO_STREAM("y:	" << error_y);
	ROS_INFO_STREAM("z:	" << error_z);
	ROS_INFO_STREAM("	Velocity commands");
	ROS_INFO_STREAM("Velocity x:	" << msg_move.linear.x);
	ROS_INFO_STREAM("Velocity y:	" << msg_move.linear.y);
	ROS_INFO_STREAM("Velocity z:	" << msg_move.linear.z);
	ROS_INFO_STREAM("yaw:	" << error_yaw << "	" << msg_move.angular.z << std::endl);

	ROS_INFO_STREAM(std::endl);
}


// ----------- Main --------------------------------------------------------
int main(int argc, char **argv){
  ros::init(argc, argv, "pose_controller_with_initialitzation");   

  pose_controller PoseController;

  ros::spin();
  return 0;
}


// ------------- Constructor -------------------------------------------------------------------
pose_controller::pose_controller(){
	sub_ardrone = n.subscribe("ardrone/navdata", 1000, &pose_controller::controlCallback,this);
	sub_pose = n.subscribe("vslam/pose", 1000, &pose_controller::poseCallback,this);
	sub_init = n.subscribe("spooler/custom", 1000, &pose_controller::initCallback,this);
	sub_waypoint = n.subscribe("spooler/cmd_aux", 1000, &pose_controller::waypointCallback,this);
	pub_move = n.advertise<ardrone_msgs::Vel>("spooler/cmd_vel", 1000);
	pub_map = n.advertise<std_msgs::String>("vslam/key_pressed", 1000);
	pub_takeoff = n.advertise<ardrone_msgs::Priority>("spooler/takeoff", 1000);

	ros::Duration time_stamp_last_msg_(0.0);

	timer_move = n.createTimer(ros::Duration(1), &pose_controller::MovingCallback, this);
	timer_lost = n.createTimer(ros::Duration(5), &pose_controller::LostCallback, this);
	timer_scale = n.createTimer(ros::Duration(2), &pose_controller::ScaleCallback, this);
	timer_wait = n.createTimer(ros::Duration(1), &pose_controller::WaitCallback, this);
	timer_x = n.createTimer(ros::Duration(1), &pose_controller::xCallback, this);
	timer_y = n.createTimer(ros::Duration(1), &pose_controller::yCallback, this);

	//Initialisation of variables
	timer_move.stop();
	timer_lost.stop();
	timer_scale.stop();
	timer_wait.stop();
	timer_x.stop();
	timer_y.stop();

	msg_move.linear.x = 0;
	msg_move.linear.y = 0;
	msg_move.linear.z = 0;
	msg_move.angular.z = 0;

	map_works_ = false;  
	first = true;
	time_x = false;
	time_y = false;
	state = TAKEOFF;
	scale = INIT_SCALING;
}
