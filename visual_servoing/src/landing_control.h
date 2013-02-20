#include <ros/ros.h>
#include <iostream>
#include <string>
#include <boost/thread.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <visual_servoing/Tag_poseAction.h>
#include <visual_servoing/trackingAction.h>

#include <ardrone_msgs/Vel.h>
#include <ardrone_msgs/Priority.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/CamSelect.h>

typedef actionlib::SimpleActionClient<visual_servoing::Tag_poseAction> detect_client;
typedef actionlib::SimpleActionClient<visual_servoing::trackingAction> track_client;

class land{

public:
	land();
  ~land(void){}

	//Functions
	void NavdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg);
	void detection_feedbackCb(const visual_servoing::Tag_poseFeedbackConstPtr& feedback);
	void detection_doneCb(const actionlib::SimpleClientGoalState& state);
	void tracking_feedbackCb(const visual_servoing::trackingFeedbackConstPtr& feedback);
	void tracking_doneCb(const actionlib::SimpleClientGoalState& state);
	void pid(float gain, float error_x, float error_y);
	void LostCallback(const ros::TimerEvent& event);
	void print();

private:
	ros::NodeHandle nh_;

	//Create the action clients
	detect_client detection;
	track_client tracking;

	//Subscribers and publishers
	ros::Subscriber sub_ardrone;
	ros::Subscriber sub_tag_feedback;
	ros::Publisher pub_takeoff;
	ros::Publisher pub_move;
	ros::Publisher pub_land;

	ardrone_msgs::Priority msg_takeoff;
	ardrone_msgs::Vel msg_move;
	ardrone_msgs::Priority msg_land;

	//To set the bottom camera
	ros::ServiceClient client;
	ardrone_autonomy::CamSelect srv;

	//Node variables
	enum {TAKEOFF = 0, SEARCHING = 1, TRACKING = 2, LOST = 3, LANDING = 4} status;
	int drone_state, t;
	float battery;
	bool searching, target_found, landing, battery_low, ai;
	bool found, foundSure, tracking_working;
	bool init_takeoff, init_detector, init_tracker;
	int altd, tag_x, tag_y;
	float vel_x, vel_y;
	float velo_x, velo_y, velo_z;
	ros::Time last_time, time;
	ros::Timer timer_lost;
	std::string info, status_info, drone;
};
