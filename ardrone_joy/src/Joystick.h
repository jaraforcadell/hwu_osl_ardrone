
#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

// ================================================================== includes

#include <ros/ros.h>
#include <ardrone_msgs/Vel.h>
#include <ardrone_msgs/Priority.h>


// ========================================================== external defines

/*
 * BUTTON MAPPING INFORMATION
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Buttons appear to be indexed from 0, but otherwise keep to the same order
 * as written on the joypad itself.  So button 1 is pad.bPos[0], etc.
 */

#define MAX_AXIS 16
#define MAX_BUTTON 16
#define DEFAULT_DEVICE_PATH "/dev/input/js0"

typedef struct {
	unsigned char axisCount;
	unsigned char buttonCount;
	int fd;
	int version;
	char devName[80];
	int aPos[MAX_AXIS];
	int bPos[MAX_BUTTON];
	bool changed;
	js_event ev;
} pad_data_t;

// ===================================================================== class
class Joystick {

public:

	Joystick(int argc, char ** argv);
	
	~Joystick();	

	int main();

private:

	void doWork();

	float scaleStick(int value, float limit);

	bool readLatestState();

	void printPadState();

	void sendVelocityMsg(float x, float y, float z, float yaw, bool takeoff, bool land, bool reset, bool takeover, bool custom, float x_aux, float y_aux);

	ros::NodeHandle nh_;
	ros::Timer doWorkTimer_;

	ros::Publisher pub_move;
	ros::Publisher pub_aux;
	ros::Publisher pub_takeoff;
	ros::Publisher pub_land;
	ros::Publisher pub_reset;
	ros::Publisher pub_endtakeover;
	ros::Publisher pub_custom;
	ardrone_msgs::Vel msg_move;
	ardrone_msgs::Vel msg_aux;
	ardrone_msgs::Priority msg_takeoff;
	ardrone_msgs::Priority msg_land;
	ardrone_msgs::Priority msg_reset;
	ardrone_msgs::Priority msg_endtakeover;
	ardrone_msgs::Priority msg_custom;
	std::string deviceName;

	pad_data_t pad;

	bool lastValuesNonZero;

};

#endif
