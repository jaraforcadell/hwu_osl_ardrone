#include "ros/ros.h"
#include "first_controller.h"

//----- Navdata Callback ---------------------------------------------------------------------------
void controller::controlCallback(const ardrone_autonomy::Navdata::ConstPtr& msg){

int drone_state = msg -> state; //state indicated by the Navdata messages from the quadcopter

if (state == LANDED){
	ardrone_msgs::Priority msg_takeoff;
	msg_takeoff.priority == false;
	pub_takeoff.publish(msg_takeoff);
	ROS_INFO_STREAM("State: LANDED");
	state=TAKING_OFF;
}
else if (state == TAKING_OFF){
	ROS_INFO_STREAM(drone_state);
	ROS_INFO_STREAM("State: TAKING_OFF");
	if(drone_state != 6 && drone_state != 2){ //state 6 is taking off and state 2 is landed
		timer_move.start();
		state=MOVING;
	}
}
else if (state == MOVING){

	msg_move.linear.x=0.08; //Moves forwards to a velocity of 0.08 m/s
	msg_move.priority=false;
	pub_move.publish(msg_move);
	ROS_INFO_STREAM("State: MOVING  -  vel_x = " << msg -> vx);
}
else if (state == STOPPING){
	msg_move.angular.z=0;
	msg_move.linear.x=0;
	msg_move.priority=false;
	pub_move.publish(msg_move);
	ROS_INFO_STREAM("State: STOPPING  -  vel_x = " << msg -> vx);
	state=WAITING;
}
else if (state == WAITING){
	ROS_INFO_STREAM("State: WAITING");
}
else if (state == START_LANDING){
	timer_wait.stop();
	ardrone_msgs::Priority msg_land;
	msg_land.priority = false;
	pub_land.publish(msg_land);
	ROS_INFO_STREAM("State: START_LANDING);
	state=LANDING;
}
else if (state == LANDING){
	ROS_INFO_STREAM("State: LANDING");
	if(drone_state == 2) // = landed
		state=FINISHED;
}
else if (state == FINISHED){
	ROS_INFO_STREAM_THROTTLE(5,"State: FINISHED. - Mission finished succesfully");
}
}

void controller::WaitingCallback(const ros::TimerEvent& event){
	state=START_LANDING;
}

void controller::MovingCallback(const ros::TimerEvent& event){
	ROS_INFO_STREAM("Timer zero");
	state=STOPPING;
	timer_move.stop();
	timer_wait.start();
}

//---------- CONSTRUCTOR -----------------------------------------------------------
controller::controller(){
	sub_data = n.subscribe("ardrone/navdata", 1000, &controller::controlCallback,this);
	pub_takeoff = n.advertise<ardrone_msgs::Priority>("spooler/takeoff", 1000);
	pub_land = n.advertise<ardrone_msgs::Priority>("spooler/land", 1000);
	pub_move = n.advertise<ardrone_msgs::Vel>("spooler/cmd_vel", 1000);
	timer_wait = n.createTimer(ros::Duration(2), &controller::WaitingCallback, this);
	timer_wait.stop();	
	waiting=false;
	timer_move = n.createTimer(ros::Duration(2), &controller::MovingCallback, this);
	timer_move.stop();
	moving=false;
	state=LANDED;
	msg_move.linear.x=0;
	msg_move.linear.y=0;
	msg_move.linear.z=0;
	msg_move.angular.z=0;
	msg_move.priority=false;
}

//------------- MAIN ------------------------------------------------------------------

int main(int argc, char **argv){

  ros::init(argc, argv, "controller"); 

	std::cout << "Waiting for any key to start the controller";
	std::cin.ignore(); // It doesn't take off until the user presses a key
	std::cin.get();
  
  controller NewController;

  ros::spin();

  return 0;
}
