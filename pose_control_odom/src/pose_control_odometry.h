#include <ros/ros.h>
#include <ardrone_msgs/Vel.h>
#include <ardrone_msgs/Priority.h>
#include <ardrone_autonomy/Navdata.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <control_toolbox/pid.h>

class pose_controller{
public:
	pose_controller();

	//Functions
	void controlCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);
	void print();

private:
	ros::NodeHandle n;

	//Subscribers and publishers
	ros::Publisher  pub_move;
	ros::Publisher  pub_takeoff;
	ros::Publisher	pub_land;
	ros::Subscriber sub_ardrone;

	ardrone_msgs::Vel msg_move; 
	ardrone_msgs::Priority msg_takeoff;
	ardrone_msgs::Priority msg_land;

	//Node variables
	enum {TAKEOFF = 0, POSITION_KEEPING = 1, LANDING = 2} state;
	double x, y, z, z_init, yaw_init, yaw, yaw_rad;
	float delta_t;
	double margin, margin_yaw;
	double x_vel, y_vel, z_vel, yaw_vel;

	control_toolbox::Pid pid_x, pid_y, pid_z, pid_yaw;
	ros::Time last_time, time;

	float battery;
	int drone_state, altd;
	std::string status, drone, info;
};
