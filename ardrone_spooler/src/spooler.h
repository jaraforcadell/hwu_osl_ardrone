#include <ros/ros.h>
#include <ardrone_msgs/Vel.h>
#include <ardrone_msgs/Priority.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/FlightAnim.h"

class spooler {

public:
	spooler();


private:
	void SendVelocityMsg(const ardrone_msgs::Vel::ConstPtr& msg);
	void SendTakeoffMsg(const ardrone_msgs::Priority::ConstPtr& msg);
	void SendLandMsg(const ardrone_msgs::Priority::ConstPtr& msg);
	void SendResetMsg(const ardrone_msgs::Priority::ConstPtr& msg);
	void EndOfTakeover(const ardrone_msgs::Priority::ConstPtr& msg);
	void LeftFlip(const ardrone_msgs::Priority::ConstPtr& msg);
	void RightFlip(const ardrone_msgs::Priority::ConstPtr& msg);
	void BackFlip(const ardrone_msgs::Priority::ConstPtr& msg);
	void ForwardFlip(const ardrone_msgs::Priority::ConstPtr& msg);
	void TimerCallback(const ros::TimerEvent& event);

	ros::NodeHandle n;

	//publishers to topics where ardrone reads
	ros::Publisher  pub_move;
	ros::Publisher  pub_takeoff;
	ros::Publisher  pub_land;
	ros::Publisher  pub_reset;

	//subscribers where both controller and joystick nodes publish
	ros::Subscriber sub_move; 
	ros::Subscriber sub_takeoff;
	ros::Subscriber sub_land;
	ros::Subscriber sub_reset;
	ros::Subscriber sub_endtakeover;
	ros::Subscriber sub_left_flip;
	ros::Subscriber sub_right_flip;
	ros::Subscriber sub_back_flip;
	ros::Subscriber sub_forward_flip;

	ros::ServiceClient client;
	ardrone_autonomy::FlightAnim srv;

	ros::Timer timer_flip;

	geometry_msgs::Twist msg_move;

	bool priority;
	bool previous_priority;
	bool doing_flip;
};
