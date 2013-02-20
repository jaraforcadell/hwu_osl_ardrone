#include "spooler.h"


//-----------------------------------------------------------------------------------

void spooler::SendVelocityMsg(const ardrone_msgs::Vel::ConstPtr& msg_vel)
{
	float linear_x= msg_vel -> linear.x;
	float linear_y= msg_vel -> linear.y;
	float linear_z= msg_vel -> linear.z;
	float angular_z= msg_vel -> angular.z;
	priority = msg_vel -> priority;

	//If previous priority is false: means we are following commands from the controller, so we can keep listening to messages without priority.

	if (priority==true || (priority==false && previous_priority==false) ){ 
		
		msg_move.linear.x= linear_x;
		msg_move.linear.y= linear_y;
		msg_move.linear.z= linear_z;
		msg_move.angular.z= angular_z;
	
		pub_move.publish(msg_move);

		previous_priority = priority;
	
		//If the message was from a controller, priority is set to false, and next message with false priority will be passed 
		
		ROS_INFO_STREAM("Moving:");
		ROS_INFO_STREAM("Vel x = " << linear_x);
		ROS_INFO_STREAM("Vel y = " << linear_y);
		ROS_INFO_STREAM("Vel z = " << linear_z);
		ROS_INFO_STREAM("Ang z = " << angular_z);
		ROS_INFO_STREAM("Priority: " << priority << std::endl);
		 
	}

}

//------------------------------------------------------------------------------------------

void spooler::SendTakeoffMsg(const ardrone_msgs::Priority::ConstPtr& msg)
{
	priority = msg -> priority;

	if(priority==true || (priority==false && previous_priority==false) ){ 
		std_msgs::Empty msg_takeoff;
		pub_takeoff.publish(msg_takeoff);
		
		previous_priority=priority;

		ROS_INFO_STREAM("Taking off.. - priority: " << priority << std::endl);

	}
}

//------------------------------------------------------------------------------------------

void spooler::SendLandMsg(const ardrone_msgs::Priority::ConstPtr& msg)
{
	priority = msg -> priority;

	if(priority==true || (priority==false && previous_priority==false) ){
		std_msgs::Empty msg_land;
		pub_land.publish(msg_land);

		previous_priority=priority;

		ROS_INFO_STREAM("Landing.. - priority:" << priority << std::endl);

	}
}

//------------------------------------------------------------------------------------------

void spooler::SendResetMsg(const ardrone_msgs::Priority::ConstPtr& msg){
	priority = msg -> priority;

	if(priority==true || (priority==false && previous_priority==false) ){		
		std_msgs::Empty msg_reset;
		pub_reset.publish(msg_reset);

		previous_priority=priority;

		ROS_INFO_STREAM("RESET!! - priority:" << priority << std::endl);

	}
}

//------------------------------------------------------------------------------------------

void spooler::EndOfTakeover(const ardrone_msgs::Priority::ConstPtr& msg){
	priority = msg -> priority;

	previous_priority = false;
	ROS_INFO_STREAM("end of joystick takeover");
	ROS_INFO_STREAM("previous priority" << previous_priority);
	ROS_INFO_STREAM("priority" << priority << std::endl);
	// Now the messages with false priority will be sent.
}

//-----------------------------------------------------------------------------------

void spooler::LeftFlip(const ardrone_msgs::Priority::ConstPtr& msg){
	priority = msg -> priority;
	if(!doing_flip){
		ROS_INFO_STREAM("Left flip" << std::endl);
		srv.request.type = 18;
		srv.request.duration = 0;
		client.call(srv);
		doing_flip=true;	
		timer_flip.start();
	}
}

//-----------------------------------------------------------------------------------

void spooler::RightFlip(const ardrone_msgs::Priority::ConstPtr& msg){
	priority = msg -> priority;
	if(!doing_flip){
		ROS_INFO_STREAM("Right flip" << std::endl);
		srv.request.type = 19;
		srv.request.duration = 0;
		client.call(srv);
		doing_flip=true;	
		timer_flip.start();
	}
}

//-----------------------------------------------------------------------------------

void spooler::BackFlip(const ardrone_msgs::Priority::ConstPtr& msg){
	priority = msg -> priority;
	if(!doing_flip){
		ROS_INFO_STREAM("Back flip" << std::endl);
		srv.request.type = 17;
		srv.request.duration = 0;
		client.call(srv);
		doing_flip=true;
		timer_flip.start();

	}

}

//-----------------------------------------------------------------------------------

void spooler::ForwardFlip(const ardrone_msgs::Priority::ConstPtr& msg){
	priority = msg -> priority;
	if(!doing_flip){
		ROS_INFO_STREAM("Forward flip" << std::endl);
		srv.request.type = 16;
		srv.request.duration = 0;
		client.call(srv);
		doing_flip=true;	
		timer_flip.start();
	}
}

//------------------------------------------------------------------------------------------

void spooler::TimerCallback(const ros::TimerEvent& event){
	doing_flip=false;
	timer_flip.stop();
}

//---------------  CONSTRUCTOR ------------------------------------------------------------
spooler::spooler() {
	sub_move = n.subscribe("spooler/cmd_vel", 1000, &spooler::SendVelocityMsg,this);
	sub_takeoff = n.subscribe("spooler/takeoff", 1000, &spooler::SendTakeoffMsg,this);
	sub_land = n.subscribe("spooler/land", 1000, &spooler::SendLandMsg,this);
	sub_reset = n.subscribe("spooler/reset", 1000, &spooler::SendResetMsg,this);
	sub_endtakeover = n.subscribe("spooler/endtakeover", 1000, &spooler::EndOfTakeover,this);
	sub_left_flip = n.subscribe("spooler/left_flip", 1000, &spooler::LeftFlip,this);
	sub_right_flip = n.subscribe("spooler/right_flip", 1000, &spooler::RightFlip,this);
	sub_back_flip = n.subscribe("spooler/back_flip", 1000, &spooler::BackFlip,this);
	sub_forward_flip = n.subscribe("spooler/forward_flip", 1000, &spooler::ForwardFlip,this);

	client = n.serviceClient<ardrone_autonomy::FlightAnim>("ardrone/setflightanimation");

	pub_move = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 
	pub_takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
	pub_land = n.advertise<std_msgs::Empty>("ardrone/land", 1000);
	pub_reset = n.advertise<std_msgs::Empty>("ardrone/reset", 1000);

	msg_move.linear.x=0;
	msg_move.linear.y=0;
	msg_move.linear.z=0;
	msg_move.angular.z=0;
	previous_priority=false;
	doing_flip=false;
	timer_flip = n.createTimer(ros::Duration(5), &spooler::TimerCallback, this);
	timer_flip.stop();	
}


//----------------------- Main ----------------------------------------------------------------

int main (int argc, char * argv[]){

	ros::init(argc, argv, "ardrone_spooler");
	
	spooler Spooler;

	ros::spin();

	return 0;
}
