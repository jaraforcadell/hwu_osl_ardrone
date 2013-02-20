#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include <ardrone_msgs/Vel.h>
#include <ardrone_msgs/Priority.h>


class controller
{

public:
	controller();
	void controlCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);
	void WaitingCallback(const ros::TimerEvent& event);
	void MovingCallback(const ros::TimerEvent& event);


private:
	ros::Publisher  pub_takeoff;
	ros::Publisher  pub_land;
	ros::Publisher  pub_move;
	ros::Subscriber sub_data;
	ros::NodeHandle n;
	
	ardrone_msgs::Vel msg_move;  

	ros::Timer timer_wait;
	ros::Timer timer_move;
	bool waiting, moving;
	int seq;
	enum {LANDED = 0, TAKING_OFF = 1, MOVING = 2, STOPPING = 3, WAITING = 4, START_LANDING = 5, LANDING = 6, FINISHED = 7} state;
};
