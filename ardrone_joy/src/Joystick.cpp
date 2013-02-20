
// Define DEBUG_OUTPUT to enable PRINTLN_DEBUG output when not using ROS.
// ROS debug level output is toggled at runtime using rxconsole.
//#define DEBUG_OUTPUT

// ================================================================== includes

#include <iostream>

using namespace std;

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/joystick.h>

#include "Joystick.h"

using namespace std;

// =================================================================== defines

#define MAX_ANALOG_VALUE 32767
#define LINEAR_LIMIT 0.3
#define ANGULAR_LIMIT 1

#define VELOCITY_SCALE_X 1
#define VELOCITY_SCALE_Y 1
#define VELOCITY_SCALE_Z 1
#define VELOCITY_SCALE_YAW 1

// =================================================================== methods
// ---------------------------------------------------------------------------
Joystick::Joystick(int argc, char ** argv)
{

if(argc==3){
	if (strcmp(argv[1], "-p") == 0)
			{
				ROS_INFO_STREAM("Using joystick device: " << argv[2]);
				deviceName = "/dev/input/" + string(argv[2]);
			}
		}

	if(deviceName.length() == 0)
	{
		deviceName = DEFAULT_DEVICE_PATH;
		ROS_INFO_STREAM("Using DEFAULT Device: " << deviceName);
	}
}
Joystick::~Joystick()
{
}
// ---------------------------------------------------------------------------
int Joystick::main(){

	ROS_INFO_STREAM("init entered");
	
	// And create a timer for periodic calls to read Joy.
	doWorkTimer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&Joystick::doWork ,this));

	memset(&pad, 0, sizeof(pad));
	pad.fd = open(deviceName.c_str(), O_RDONLY);
	if (pad.fd <= 0)
	{
		close(pad.fd);
		cerr << "Failed to open joystick - check it is attached." << endl;
		return false;
	}

	cout << "Joystick opened ok." << endl;

	// Get pad info ...
	ioctl(pad.fd, JSIOCGAXES, &pad.axisCount);
	ioctl(pad.fd, JSIOCGBUTTONS, &pad.buttonCount);
	ioctl(pad.fd, JSIOCGVERSION, &pad.version);
	ioctl(pad.fd, JSIOCGNAME(80), &pad.devName);
	fcntl(pad.fd, F_SETFL, O_NONBLOCK);

	cout << "axis : " << (int)pad.axisCount << endl;
	cout << "buttons : " << (int)pad.buttonCount << endl;
	cout << "version : " << pad.version << endl;
	cout << "name : " << pad.devName << endl;

	// set default values
	pad.changed = false;
	for (int i=0;i<pad.axisCount;i++) pad.aPos[i]=0;
	for (int i=0;i<pad.buttonCount;i++) pad.bPos[i]=0;

	lastValuesNonZero = true;

	pub_move = nh_.advertise<ardrone_msgs::Vel>("spooler/cmd_vel", 1);
	pub_aux = nh_.advertise<ardrone_msgs::Vel>("spooler/cmd_aux", 1);
	pub_takeoff = nh_.advertise<ardrone_msgs::Priority>("spooler/takeoff", 1000);
	pub_land = nh_.advertise<ardrone_msgs::Priority>("spooler/land", 1000);
	pub_reset = nh_.advertise<ardrone_msgs::Priority>("spooler/reset", 1000);
	pub_endtakeover=nh_.advertise<ardrone_msgs::Priority>("spooler/endtakeover",1000);
	pub_custom = nh_.advertise<ardrone_msgs::Priority>("spooler/custom", 1000);

	ROS_INFO_STREAM("initialisation complete");

	while(ros::ok()){
		ros::spin();	
	}
	
	close(pad.fd);
	return true;

}
void Joystick::doWork()
{
	readLatestState();
	//printPadState();  // Uncomment to know which axis is each button
	float x, y, z, yaw, x_aux, y_aux;
	bool takeoff, land, reset, endtakeover, custom;
	x = scaleStick(-pad.aPos[3], LINEAR_LIMIT);
	y = scaleStick(-pad.aPos[2], LINEAR_LIMIT);
	z = scaleStick(-pad.aPos[1], LINEAR_LIMIT);
	yaw = scaleStick(-pad.aPos[0], ANGULAR_LIMIT);
	takeoff = pad.bPos[3];
	land = pad.bPos[1];
	reset = pad.bPos[9];
	endtakeover = pad.bPos[8];
	custom = pad.bPos[12];
	x_aux = scaleStick(-pad.aPos[5], LINEAR_LIMIT);
	y_aux = scaleStick(-pad.aPos[4], LINEAR_LIMIT);


	bool sending = true;

	if (x != 0 || y != 0 || z != 0 || yaw != 0 || takeoff != 0 || land != 0 || reset != 0 )
	{
		ROS_INFO_STREAM(-pad.aPos[3] << " " << -pad.aPos[2] << " " << -pad.aPos[1] << " " << -pad.aPos[0] << " ");
		ROS_INFO_STREAM("Joystick forces non-zero, sending msg.");
		lastValuesNonZero = true;
	}
	else if (lastValuesNonZero)
	{
		ROS_INFO_STREAM("Sending last zero msg, then stopping.");
		lastValuesNonZero = false;
	}
	else {
		ROS_INFO_STREAM("Joystick forces zero. Not sending");
		sending = false;
	}
	
	if (endtakeover != 0){
		ROS_INFO_STREAM("Sending end of take over msg.");
		sendVelocityMsg(x, y, z, yaw, takeoff, land, reset, endtakeover, custom, x_aux, y_aux);
		sending = false;
	}

	if (custom != 0 or x_aux!=0 or y_aux!=0){
		sendVelocityMsg(x, y, z, yaw, takeoff, land, reset, endtakeover, custom, x_aux, y_aux);
		sending = false;
	}
	
	if(sending){
		sendVelocityMsg(x, y, z, yaw, takeoff, land, reset, endtakeover, custom, x_aux, y_aux);
	}

	

}

// ---------------------------------------------------------------------------
float Joystick::scaleStick(int value, float limit)
{
  if(abs(value) < 2000) value=0;
  //cout << "Analog value: " << value << " Limit value: " << limit << " Max analog value: " << MAX_ANALOG_VALUE << endl;
	return value * limit / MAX_ANALOG_VALUE;
}

// ---------------------------------------------------------------------------
bool Joystick::readLatestState()
{
	pad.changed = false;
	int result;
	while ((result = read(pad.fd, &pad.ev, sizeof(pad.ev))) > 0)
	{
		switch (pad.ev.type)
		{
		case JS_EVENT_INIT:
		case JS_EVENT_INIT | JS_EVENT_AXIS:
		case JS_EVENT_INIT | JS_EVENT_BUTTON:
		break;
		case JS_EVENT_AXIS:
		pad.aPos[pad.ev.number] = pad.ev.value;
		pad.changed = true;
		break;
		case JS_EVENT_BUTTON:
		pad.bPos[pad.ev.number] = pad.ev.value;
		pad.changed = true;
		break;
		default:
		printf("Other event ? %d\nnumber=%d\nvalue=%d\n",
		pad.ev.type,pad.ev.number, pad.ev.value);
		break;
		}
	}

	if (errno != EAGAIN) {
		cerr << "Aborting on joystick read error: " << errno << endl;
	//	requestRestart();
	}

	return pad.changed;
}

// ---------------------------------------------------------------------------
void Joystick::printPadState()
{
	cout << "----------------------------------------------" << endl;
	cout << "pad.changed: " << pad.changed << endl;
	cout << "axis : " << pad.axisCount << endl;
	cout << "buttons : " << pad.buttonCount << endl;
	cout << "version : " <<  pad.version << endl;
	cout << "name : " << pad.devName << endl;
	cout << "----------------------------------------------" << endl;
	cout << "last ev time : " << pad.ev.time << endl;

	for (int i=0;i<pad.axisCount;i++) printf(" Axis %2d |",i);
	printf("\n");
	for (int i=0;i<pad.axisCount;i++) printf(" %7d |",pad.aPos[i]);
	printf("\n\n");
	for (int i=0;i<pad.buttonCount;i++) printf(" Btn.%2d |",i);
	printf("\n");
	for (int i=0;i<pad.buttonCount;i++) printf("     %2d |",pad.bPos[i]);
	printf("\n");
}



// ---------------------------------------------------------------------------
void Joystick::sendVelocityMsg(float x, float y, float z, float yaw, bool takeoff, bool land, bool reset, bool endtakeover, bool custom, float x_aux, float y_aux)
{
	msg_move.linear.x = VELOCITY_SCALE_X*x;
	msg_move.linear.y = VELOCITY_SCALE_X*y;
	msg_move.linear.z = VELOCITY_SCALE_X*z;
	msg_move.angular.z = VELOCITY_SCALE_YAW*yaw;
	msg_move.priority=true;

	pub_move.publish(msg_move);
	ROS_INFO_STREAM(msg_move);
	
	if(x_aux != 0 || y_aux != 0){
		msg_aux.priority=true;
		msg_aux.linear.x = VELOCITY_SCALE_X*x_aux;
		msg_aux.linear.y = VELOCITY_SCALE_X*y_aux;
		pub_aux.publish(msg_aux);
		msg_endtakeover.priority=true;
		pub_endtakeover.publish(msg_endtakeover);
		ROS_INFO_STREAM("SENDING AUX " << msg_aux << std::endl);
	}

	if (takeoff == 1){
		msg_takeoff.priority=true;
		pub_takeoff.publish(msg_takeoff);
		ROS_INFO_STREAM("taking off...");
	}

	if (land == 1){
		msg_land.priority=true;
		pub_land.publish(msg_land);
		ROS_INFO_STREAM("landing...");
	}

	if (reset == 1){
		msg_reset.priority=true;
		pub_reset.publish(msg_reset);
		ROS_INFO_STREAM("RESET");
	}

	if (endtakeover == 1){
		msg_endtakeover.priority=true;
		pub_endtakeover.publish(msg_endtakeover);
		ROS_INFO_STREAM("END OF JOYSTICK TAKEOVER");
	}

	if (custom == 1){
		msg_endtakeover.priority=true;
		pub_endtakeover.publish(msg_endtakeover);
		msg_custom.priority=true;
		pub_custom.publish(msg_custom);
		ROS_INFO_STREAM("END OF JOYSTICK TAKEOVER - CUSTOM");
	}

}

// ===========================================================================
int main (int argc, char * argv[])
{
	
	ros::init(argc, argv, "ardrone_joy");

	Joystick ardrone_joy(argc, argv);

	return ardrone_joy.main();

}
