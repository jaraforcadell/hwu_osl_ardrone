#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ardrone_msgs/Vel.h>
#include <ardrone_msgs/Priority.h>
#include <ardrone_autonomy/Navdata.h>

class pose_controller{

public:
	pose_controller();

	//Functions
	void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void controlCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);
	void initCallback(const ardrone_msgs::Priority::ConstPtr& msg);
	void waypointCallback(const ardrone_msgs::Vel::ConstPtr& msg);
	void MovingCallback(const ros::TimerEvent& event);
	void LostCallback(const ros::TimerEvent& event);
	void ScaleCallback(const ros::TimerEvent& event);
	void WaitCallback(const ros::TimerEvent& event);
	void xCallback(const ros::TimerEvent& event);
	void yCallback(const ros::TimerEvent& event);
	void print();

private:
	ros::NodeHandle n;

	//Subscribers and publishers
	ros::Publisher  pub_move;
	ros::Publisher  pub_map;
	ros::Publisher  pub_takeoff;
	ros::Subscriber sub_pose;
	ros::Subscriber sub_ardrone;
	ros::Subscriber sub_init;
	ros::Subscriber sub_waypoint;
	ros::Timer timer_lost;
	ros::Timer timer_move;
	ros::Timer timer_scale;
	ros::Timer timer_wait;
	ros::Timer timer_x;
	ros::Timer timer_y;
	ardrone_msgs::Vel msg_move; 
	ardrone_msgs::Priority msg_takeoff;

	//Node variables
	ros::Time time_stamp_last_msg_;
	bool first, map_works_, time_x, time_y;
	float alt, alt0, alt1, alt_diff, z0, z1, alt_diff_ptam, sc;
	double gain;

	float x, y, z, goal_x, goal_y, goal_z, goal_yaw;
	float error_x, error_y, error_z, error_yaw;
	float margin, big_margin;

	int drone_state;

	btScalar roll, pitch, yaw;
	btQuaternion q;

	std::string info, current_scale, current_state, drone;
	
	enum {TAKEOFF = 0, START_INIT = 1, MOVING_TO_INIT = 2, END_INIT = 3, WAITING = 4, RE_INIT = 5, POSITION_KEEPING = 6, LOST = 7} state;
	enum {INIT_SCALING = 0, MOVING = 1, MOVED = 2, END_SCALING = 3, GO_DOWN = 4, SCALED = 5} scale;
};
