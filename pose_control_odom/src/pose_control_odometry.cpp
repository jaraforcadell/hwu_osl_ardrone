#include "pose_control_odometry.h"

//------ Control callback ----------------------------------------------------------------
//To start the tracking when is required by the master
void pose_controller::controlCallback(const ardrone_autonomy::Navdata::ConstPtr& msg){
	//Get the ardrone state, the battery percent and the altitude
	drone_state = msg -> state;
	battery = msg -> batteryPercent;
	altd = msg -> altd;

	//Check the battery
	if (battery <= 6){
		status = LANDING;
		info = "BATTERY LOW";
	}

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
	
	if (state == TAKEOFF){
		status = "TAKE OFF";
		info = "Taking off";
		msg_takeoff.priority = false;
		pub_takeoff.publish(msg_takeoff);
		if(drone_state == 4){
			//Initialise zero position
			x = 0;
			y = 0;
			z_init = msg->altd;
			yaw = 0;
			yaw_init = msg->rotZ;

			//Change state
			state = POSITION_KEEPING;
			last_time = msg->header.stamp;
		} 
	} else if (state == POSITION_KEEPING){
		status = "POSITION KEEPING";
		info = "Doing position keeping";

		//Now update navigation information based on received data from ardrone odometry
		time = msg->header.stamp;
		delta_t = ((time.sec + time.nsec/10e9) - (last_time.sec + last_time.nsec/10e9));
		yaw = msg->rotZ - yaw_init;
		yaw_rad = yaw/180*M_PI;
		x = x + msg->vx * (delta_t/1000);
		y = y + msg->vy * (delta_t/1000);
		z = msg->altd - z_init;
		z = z / 1000;

		margin = 0.1;
		margin_yaw = 0.4;

		//Yaw control
		if (fabs(yaw_rad) > margin_yaw){
			yaw_vel = pid_yaw.updatePid(yaw, time - last_time);
			msg_move.angular.z = yaw_vel;
			info = "Correcting position";
		} else {
			msg_move.angular.z = 0;
		}

		//X control
		if (fabs(x) > margin && fabs(yaw_rad) < margin_yaw){
			x_vel = pid_x.updatePid(x, time - last_time);
			msg_move.linear.x = x_vel;
			info = "Correcting position";
		} else {
			msg_move.linear.x = 0;
		}

		//Y control
		if (fabs(y) > margin && fabs(yaw_rad) < margin_yaw){
			y_vel = pid_y.updatePid(y, time - last_time);
			msg_move.linear.y = y_vel;
			info = "Correcting position";
		} else {
			msg_move.linear.y = 0;
		}

		//Altitude control
		if (fabs(z) > margin){
			z_vel = pid_z.updatePid(z, time - last_time);
			msg_move.linear.z = z_vel;
			info = "Correcting position";
		} else {
			msg_move.linear.z = 0;
		}

		//Publish velocity messages:
		msg_move.priority = false;
		pub_move.publish(msg_move);

		last_time = msg->header.stamp;

	} else if (state == LANDING){
		status = "LANDING";
		msg_land.priority = true;
		pub_land.publish(msg_land);
	}

	print();
}


// --------- Print function -------------------------------------------------------------
void pose_controller::print(){
		float sec = 0.12;

		ROS_INFO_STREAM_THROTTLE(sec, "--------------ODOMETRY CONTROL--------------");
		ROS_INFO_STREAM_THROTTLE(sec, "Code state:	" << status);
		ROS_INFO_STREAM_THROTTLE(sec, "Battery:	" << battery << "%");
		ROS_INFO_STREAM_THROTTLE(sec, "Drone state:	" << drone);
		ROS_INFO_STREAM_THROTTLE(sec, "Altitude:	" << altd);
		ROS_INFO_STREAM_THROTTLE(sec, "Info: 		" << info);

		ROS_INFO_STREAM_THROTTLE(sec, "------------------CONTROL------------------");
		ROS_INFO_STREAM_THROTTLE(sec, "		Velocities");
		ROS_INFO_STREAM_THROTTLE(sec, "		----------");
		ROS_INFO_STREAM_THROTTLE(sec, "Velocity x:	" << msg_move.linear.x);
		ROS_INFO_STREAM_THROTTLE(sec, "Velocity y:	" << msg_move.linear.y);
		ROS_INFO_STREAM_THROTTLE(sec, "Velocity z:	" << msg_move.linear.z);
		ROS_INFO_STREAM_THROTTLE(sec, "Yaw:		" << msg_move.angular.z);
		ROS_INFO_STREAM_THROTTLE(sec, "-------------------------------------------" << std::endl);

		ROS_INFO_STREAM_THROTTLE(sec, std::endl);
}


// -------- Main --------------------------------------------------------
int main(int argc, char **argv){
  ros::init(argc, argv, "pose_controller_with_initialitzation");   

  pose_controller PoseController;

  ros::spin();
  return 0;
}


// ------- Constructor ------------------------------------------------------------------------
pose_controller::pose_controller(){
	sub_ardrone = n.subscribe("ardrone/navdata", 1000, &pose_controller::controlCallback,this);
	pub_move = n.advertise<ardrone_msgs::Vel>("spooler/cmd_vel", 1000);
	pub_takeoff = n.advertise<ardrone_msgs::Priority>("spooler/takeoff", 1000);
	ros::Duration time_stamp_last_msg_(0.0);

	//Initialisation of variables
	//Initialisation of pid's
	pid_x.initPid(0.2, 0.1, 0.1, 0.1, -0.1);
	pid_y.initPid(0.2, 0.1, 0.1, 0.1, -0.1);
	pid_z.initPid(0.2, 0.1, 0.1, 0.1, -0.1);
	pid_yaw.initPid(0.2, 0.1, 0.1, 0.1, -0.1);

	//Set all velocities to 0
	msg_move.linear.x = 0;
	msg_move.linear.y = 0;
	msg_move.linear.z = 0;
	msg_move.angular.z = 0;
	
	state = TAKEOFF;
}
