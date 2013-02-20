#include "tag_detection_server.h"

namespace enc = sensor_msgs::image_encodings;

// ---------- Goal callback ----------------------------------------------------------------------
//to start the target searching when required for the master
void TagDetection::goalCB(){
	start = true;

	Server_.acceptNewGoal();
	//Make sure the goal hasn't been cancelled between it was found and we accept it, as between his time period preemt callback is not triggered
	if(Server_.isPreemptRequested()){
		preemptCB();
	}
}

// ------------- Preempt callback ----------------------------------------
//to cancel the goal when required by the master
void TagDetection::preemptCB(){
	Server_.setPreempted();
	restart = false;
}


// --------- Camera image callback --------------------------------------------------------------- 
//contains the images
void TagDetection::image_cam(const sensor_msgs::ImageConstPtr& msg){
	if (start){

		//If the restart of the searching is required, set all the variables to its initial value
		if (restart){
			start = false;
			try_next = 1;
			trying = 5;
			found = false;
			found_sure = 0;
			foundSure = false;
			start = false;
			templ_loaded = false;
			restart = false;

			} else{
			//Get a frame
			cv_bridge::CvImagePtr cv_ptr;
			try{
				cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("tag_detection_server: cv_bridge exception: %s", e.what());
				return;
			}
			frame = cv_ptr->image;

			//Call the function to find the squares
			findSquares(frame, squares);

			//Show the detected squares
			/*fr_squares = frame.clone();
			drawSquares(fr_squares, squares);
			imshow("Squares detection", fr_squares);*/

			//Call the function to compare the template with the found squares
			findTarget(frame, squares, rotated, templ);

			//Show me what you got
			imshow("Target detection", frame);
			//imshow("Square normalized", rotated);
			//imshow("Result", result);

			if (!found){
				found_sure = 0;
				foundSure = false;
			}

			//Send the feedback to the master
			feedback_.found = found;
			feedback_.foundSure = foundSure;
			feedback_.tag_x = tag_x - frame.size().width/2;
			feedback_.tag_y = tag_y - frame.size().height/2;
			Server_.publishFeedback(feedback_);

			//To publish the video
			cv_bridge::CvImagePtr cv_ptr_out(new cv_bridge::CvImage);
			cv_ptr_out->image = frame;
			cv_ptr_out->encoding = "bgr8";
			sensor_msgs::Image message;
			cv_ptr_out->toImageMsg(message);
			image_pub_.publish(message);

			waitKey(3);
		}
	}
}


// ---------- findSquares function -----------------------------------------------------------------------------
//Returns a sequence of squares detected on the image. The sequence is stored in the specified memory storage
void TagDetection::findSquares(const Mat& image, vector<vector<Point> >& squares){
	squares.clear();

	Mat pyr, timg, gray0(image.size(), CV_8U), gray;
	vector<vector<Point> > contours;

	//Down-scale and upscale the image to filter out the noise
	pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
	pyrUp(pyr, timg, image.size());

	//Find squares in only one color plane of the image
	int ch[] = {1, 0};
	mixChannels(&timg, 1, &gray0, 1, ch, 1);

	//Apply Canny. Take the upper threshold 50 and set the lower to 0 (which forces edges merging)
	Canny(gray0, gray, 0, 50, 5);
	//Dilate canny output to remove potential holes between edge segments
	dilate(gray, gray, Mat(), Point(-1,-1));

	//Find contours and store them all as a list
	findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	vector<Point> approx;

	//Test each contour
	for(size_t i = 0; i < contours.size(); i++){
		//Approximate contour with accuracy proportional to the contour perimeter
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

		//Square contours should have 4 vertices after approximation relatively large area (to filter out noisy contours) be convex, and be smaller than the 80% of the frame area
		//Note: absolute value of an area is used because area may be positive or negative - in accordance with the contour orientation
		if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && contourArea(Mat(approx))<image.size().width*image.size().height*0.8 && isContourConvex(Mat(approx))){

			double maxCosine = 0;

			for( int j = 2; j < 5; j++ ){
				//Find the maximum cosine of the angle between joint edges
				double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
				maxCosine = MAX(maxCosine, cosine);
			}

			//If cosines of all angles are small (all angles are ~90 degree) then write quandrange vertices to resultant sequence
			if(maxCosine < 0.1)
				{ squares.push_back(approx); }
		}
	}
}


// ----- Angle function  --------------------------------------------------------------------
//to check if the countours found in the findSquares function
double TagDetection::angle(Point pt1, Point pt2, Point pt0){
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;

	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

//------- draWSquares function ---------------------------------------------------------------------
//Drows all the squares in the image
void TagDetection::drawSquares( Mat& image, const vector<vector<Point> >& squares ){
	for(size_t i = 0; i < squares.size(); i++){
		const Point* p = &squares[i][0];
		int n = (int)squares[i].size();
		polylines(image, &p, &n, 1, true, Scalar(0,255,0), 1, CV_AA);
	}
}

//-------  findTarget function ---------------------------------------------------------------------
//Compare the squares with the template
void TagDetection::findTarget(Mat& image, const vector<vector<Point> >& square, Mat& rotated, Mat& templ){
	//Load the template
	if (templ_loaded == false){
		templ = imread("/home/marc/electric_workspace/visual_servoing/templ_H.jpg", 1);
		if (templ.empty()){
			cout << "tag_detection_server: Cannot open the target image" << endl;
		}
		resize(templ, templ, Size(), 1, 1, INTER_NEAREST);
		templ_loaded = true;
	}

	if (square.empty()){
		found = false;

	} else{
		//Normalize the squares detected
		for(size_t i = 0; i < square.size(); i++){

			//Get the four points of the square
			vector<Point> orig_square = square[i];

			vector<Point> not_a_rect_shape;
			not_a_rect_shape.push_back(Point(orig_square[0]));
			not_a_rect_shape.push_back(Point(orig_square[1]));
			not_a_rect_shape.push_back(Point(orig_square[2]));
			not_a_rect_shape.push_back(Point(orig_square[3]));

			RotatedRect box = minAreaRect(Mat(not_a_rect_shape));

			box.points(pts);

			if (try_next == 1){
				src_vertices[0] = pts[2];
				src_vertices[1] = pts[3];
				src_vertices[2] = pts[1];
				src_vertices[3] = pts[0];
			} else if (try_next == 2){
				src_vertices[0] = pts[1];
				src_vertices[1] = pts[2];
				src_vertices[2] = pts[0];
				src_vertices[3] = pts[3];
			} else if (try_next == 3){
				src_vertices[0] = pts[0];
				src_vertices[1] = pts[1];
				src_vertices[2] = pts[3];
				src_vertices[3] = pts[2];
			} else if (try_next == 4){
				src_vertices[0] = pts[3];
				src_vertices[1] = pts[0];
				src_vertices[2] = pts[2];
				src_vertices[3] = pts[1];
			}

			Point2f dst_vertices[4];
			dst_vertices[0] = Point(0, 0);
			dst_vertices[1] = Point(box.boundingRect().width-1, 0);
			dst_vertices[2] = Point(0, box.boundingRect().height-1);
			dst_vertices[3] = Point(box.boundingRect().width-1, box.boundingRect().height-1);

			//Normalise the square. Resize it the square detected
			warpAffineMatrix = getAffineTransform(src_vertices, dst_vertices);
			Size size(box.boundingRect().width, box.boundingRect().height);
			warpAffine(image, rotated, warpAffineMatrix, size, INTER_LINEAR, BORDER_CONSTANT);

			resize(rotated, rotated, templ.size(), 1, 1, INTER_NEAREST);
			resize(rotated, rotated, Size(), 2.2, 2.2, INTER_NEAREST);

			//Call the MatchingMetod to search the template in the resized square
			MatchingMethod(0,0);

			if(found == true){ break; }
		}
	}
}


// ---------- function MatchingMethod -------------------------------------------------
void TagDetection::MatchingMethod(int, void*){
	if (rotated.empty()){
		ROS_INFO_THROTTLE(2, "tag_detection_server: No possible template in frame");
		found = false;

	} else{
		img_display = frame.clone();

		//Create the result matrix
		int result_cols =  rotated.cols - templ.cols + 1;
		int result_rows = rotated.rows - templ.rows + 1;

		result.create( result_rows, result_cols, CV_32FC1 );

		//Do the Matching and Normalize. Search the template in the rotated frame
		matchTemplate(rotated, templ, result, CV_TM_SQDIFF);
		normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

		//Localizing the best match with minMaxLoc
		double minVal, maxVal;
		Point minLoc, maxLoc, matchLoc;

		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
		matchLoc = minLoc;

		int x = matchLoc.x;
		int y = matchLoc.y;

		found = false;

		//Check if there is the possibility to find the template inside the detected square. If there is the template, the matchLoc point should be in the centre of the result window:
		if (x > result.cols/3 && x < result.cols*2/3 && y > result.rows/5 && y < result.rows*4/5){
			rectangle(result, Point(result.cols*2/5, result.rows*3/7), Point(result.cols*3/5, result.rows*4/7), Scalar(0,0,0), 2, 8, 0);
			//rectangle(result, Point(result.cols/3, result.rows/5), Point(result.cols*2/3, result.rows*4/5), Scalar(0,0,0), 2, 8, 0);
			possible = true;
			//Possible target!

			if (x > result.cols*2/5 && x < result.cols*3/5 && y > result.rows*3/7 && y < result.rows*4/7){
				found = true;
				//Target found!!
			}
		}

		//If it is found, wait a bit and draw a green rectangle in the target, if not, try to rotated it if there is a possible target
		if (found == true){
			//Find it during 5 consecutives frames to set founSure true
			found_sure++;
			if (found_sure >= 5 || (trying < 5 && trying > 0)){
				foundSure = true;
			} else{
				foundSure = false;
			}

			//Draw a rectangle in the target
			if (foundSure == false){
				//Show me what you got
				rectangle(rotated, matchLoc, Point(x + templ.cols, y + templ.rows), Scalar(0,150,255), 2, 8, 0);

				//Draw an orange rectangle in the target
				line(frame, src_vertices[0], src_vertices[1], Scalar(0,150,255), 5, 8, 0);
				line(frame, src_vertices[1], src_vertices[3], Scalar(0,150,255), 5, 8, 0);
				line(frame, src_vertices[3], src_vertices[2], Scalar(0,150,255), 5, 8, 0);
				line(frame, src_vertices[2], src_vertices[0], Scalar(0,150,255), 5, 8, 0);

			} else{
				//Show me what you got
				rectangle(rotated, matchLoc, Point(x + templ.cols, y + templ.rows), Scalar(0,255,0), 2, 8, 0);

				//Draw a green rectangle in the target
				line(frame, src_vertices[0], src_vertices[1], Scalar(0,255,0), 5, 8, 0);
				line(frame, src_vertices[1], src_vertices[3], Scalar(0,255,0), 5, 8, 0);
				line(frame, src_vertices[3], src_vertices[2], Scalar(0,255,0), 5, 8, 0);
				line(frame, src_vertices[2], src_vertices[0], Scalar(0,255,0), 5, 8, 0);

				//Find the centre of the target
				if (src_vertices[0].x < src_vertices[3].x){
					d_x = src_vertices[3].x - src_vertices[0].x;
					tag_x = src_vertices[0].x + d_x/2;
				} else{
					d_x = src_vertices[0].x-src_vertices[3].x;
					tag_x = src_vertices[3].x + d_x/2;
				}

				if (src_vertices[0].y < src_vertices[3].y){
					d_y = src_vertices[3].y-src_vertices[0].y;
					tag_y = src_vertices[0].y + d_y/2;
				} else{
					d_y = src_vertices[0].y-src_vertices[3].y;
					tag_y = src_vertices[3].y + d_y/2;
				}

			//Set the params to send the centre of the target to the tracking node
			nh_.setParam("/target/x", tag_x);
			nh_.setParam("/target/y", tag_y);

			//Set the action state to succeeded
			Server_.setSucceeded(result_);
			restart = true;
			}

		//If found is false but possible is true
		} else if(possible == true){
			//If there is no matching target, try to rotate the image
			if (try_next == 4)
				{try_next = 1;} else
				{try_next = try_next + 1;}
			trying++;
		}

		return;
	}
}


// --------------- Main -------------------------------------------------
int main(int argc, char** argv){
	ros::init(argc, argv, "visual_servoing");

	TagDetection Target_detection;

	ros::spin();
	return 0;
}


// --------------- Constructor -----------------------------------------------------------------------
TagDetection::TagDetection():
it_(nh_),
Server_(nh_,"target_detection", false)
{
	//Register the Goal and the Preept Callbacks
	Server_.registerGoalCallback(boost::bind(&TagDetection::goalCB,this));
	Server_.registerPreemptCallback(boost::bind(&TagDetection::preemptCB,this));

	image_sub_ = it_.subscribe("ardrone/front/image_rect_color", 1, &TagDetection::image_cam, this);
	image_pub_ = it_.advertise("detecting/image", 1);

	//Start the server
	Server_.start();

	//Initialisation of variables
	try_next = 1;
	trying = 5;
	found = false;
	foundSure = false;
	start = false;
	templ_loaded = false;
	restart = false;
}
