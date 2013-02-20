#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"

#include <actionlib/server/simple_action_server.h>
#include <visual_servoing/Tag_poseAction.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;
typedef actionlib::SimpleActionServer<visual_servoing::Tag_poseAction> TagServer;

class TagDetection{

public:
	TagDetection();
  ~TagDetection(void){}

	//Functions
	void goalCB();
	void preemptCB();
	void image_cam(const sensor_msgs::ImageConstPtr& msg);
	double angle( Point pt1, Point pt2, Point pt0 );
	void findSquares( const Mat& image, vector<vector<Point> >& squares );
	void drawSquares( Mat& image, const vector<vector<Point> >& squares );
	void findTarget( Mat& image, const vector<vector<Point> >& square, Mat& rotated, Mat& templ );
	void MatchingMethod( int, void* );

protected:
	ros::NodeHandle nh_;

	//Subscribers and publishers
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	//Create the server and messages that are used to published feedback/result
	TagServer Server_;
  visual_servoing::Tag_poseFeedback feedback_;
	visual_servoing::Tag_poseResult result_;

private:
	//Node variables
	bool start, restart, templ_loaded, possible, found, foundSure;
	int try_next, found_sure, trying;
	double d_x, d_y, tag_x, tag_y;

	vector<vector<Point> > squares;
	Mat frame, fr_squares, rotated, templ, img_display, result, gray_frame, warpAffineMatrix;
	Point2f src_vertices[4], pts[4];
};
