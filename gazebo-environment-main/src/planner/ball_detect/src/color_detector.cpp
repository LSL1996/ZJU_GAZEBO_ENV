#include "color_detector.h"

#define PI M_PI
#define EPSILON 1e-9

std::string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}
std::string floatToString(float number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}
bool cmp_max(cv::Moments x,cv::Moments y){
	return x.m00 > y.m00;
}

void ColorDetector::drawObject(int x, int y, float depth, cv::Mat &frame)
{
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!
	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)
	cv::circle(frame,cv::Point(x,y),20,cv::Scalar(0,255,0),2);
	if(y-25>0)
		cv::line(frame,cv::Point(x,y),cv::Point(x,y-25),cv::Scalar(0,255,0),2);
	else 
		cv::line(frame,cv::Point(x,y),cv::Point(x,0),cv::Scalar(0,255,0),2);
	if(y+25<IMG_HEIGHT_)
		cv::line(frame,cv::Point(x,y),cv::Point(x,y+25),cv::Scalar(0,255,0),2);
	else 
		cv::line(frame,cv::Point(x,y),cv::Point(x,IMG_HEIGHT_),cv::Scalar(0,255,0),2);
	if(x-25>0)
		cv::line(frame,cv::Point(x,y),cv::Point(x-25,y),cv::Scalar(0,255,0),2);
	else 
		cv::line(frame,cv::Point(x,y),cv::Point(0,y),cv::Scalar(0,255,0),2);
	if(x+25<IMG_WIDTH_)
		cv::line(frame,cv::Point(x,y),cv::Point(x+25,y),cv::Scalar(0,255,0),2);
	else 
		cv::line(frame,cv::Point(x,y),cv::Point(IMG_WIDTH_,y),cv::Scalar(0,255,0),2);
	cv::putText(frame,intToString(x)+","+intToString(y)+","+floatToString(depth),cv::Point(x,y+30),1,1,cv::Scalar(0,255,0),2);
}

void ColorDetector::init(ros::NodeHandle& nh){
	readParameters(nh);
	// checkParams();
	registerSubAndPub(nh);
	if(USE_TRACKBAR_)
		createTrackbars();
}

void ColorDetector::readParameters(ros::NodeHandle& nh)
{
	ROS_WARN("readParameters() start!!!!");
	// camera params
	nh.getParam("cam_width",  IMG_WIDTH_);
	nh.getParam("cam_height", IMG_HEIGHT_);
	nh.getParam("cam_fx", FX_);
	nh.getParam("cam_fy", FY_);
	nh.getParam("cam_cx", CX_);
	nh.getParam("cam_cy", CY_);

	// HSV params
	nh.getParam("HSV_RANGE/H_MIN", H_MIN_);
	nh.getParam("HSV_RANGE/H_MAX", H_MAX_);	
	nh.getParam("HSV_RANGE/S_MIN", S_MIN_);
	nh.getParam("HSV_RANGE/S_MAX", S_MAX_);
	nh.getParam("HSV_RANGE/V_MIN", V_MIN_);	
	nh.getParam("HSV_RANGE/V_MAX", V_MAX_);

	// debug params
	nh.getParam("DEBUG/IS_SHOW", IS_SHOW_);
	nh.getParam("DEBUG/USE_TRACKBAR", USE_TRACKBAR_);

	nh.getParam("USE_MORPHOPS", USE_MORPHOPS_);
	nh.getParam("USE_TRACK", USE_TRACK_);
	nh.getParam("MAX_NUM_OBJECTS", MAX_NUM_OBJECTS_);
	nh.getParam("MIN_OBJECT_AREA", MIN_OBJECT_AREA_);
	nh.getParam("MAX_OBJECT_AREA", MAX_OBJECT_AREA_);

	nh.param("detector/is_simulator", is_simulator_, false);

	cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

	ROS_WARN("readParameters() end!!!!");
}

void ColorDetector::registerSubAndPub(ros::NodeHandle& nh)
{
	ROS_WARN("registerSubAndPub() start!!!!");
	depth_aligned_img_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/aligned_depth_to_color/image_raw", 15, ros::TransportHints().tcpNoDelay()));
	color_img_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/color/image_raw", 15, ros::TransportHints().tcpNoDelay()));
	sync_depth_color_img_.reset(new message_filters::Synchronizer<SyncPolicyDepthColorImage>(SyncPolicyDepthColorImage(15), *depth_aligned_img_sub_, *color_img_sub_));
	
	sync_depth_color_img_->registerCallback(boost::bind(&ColorDetector::depthColorImgCallback, this, _1, _2));
	
	ROS_WARN("registerSubAndPub() end!!!!");
	
	odom_sub_ = nh.subscribe("odom_world", 1, &ColorDetector::odometryCallback, this);
	extrinsic_sub_ = nh.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &ColorDetector::extrinsicCallback, this);

	if (is_simulator_)
		dynamic_sub_ = nh.subscribe("/dynamic/pose_0", 1, &ColorDetector::dynamicCallback, this);

	arrow_pub_ 			= nh.advertise<visualization_msgs::Marker>("arrows_marker", 0);
	predict_show_pub_   = nh.advertise<visualization_msgs::Marker>("predict_show_marker", 0);

}

void ColorDetector::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  cam2body_(0, 3) = odom->pose.pose.position.x;
  cam2body_(1, 3) = odom->pose.pose.position.y;
  cam2body_(2, 3) = odom->pose.pose.position.z;
  cam2body_(3, 3) = 1.0;
}

void ColorDetector::odometryCallback(const nav_msgs::OdometryConstPtr &odom){
	/* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * cam2body_;
  camera_pos_(0) = cam_T(0, 3);
  camera_pos_(1) = cam_T(1, 3);
  camera_pos_(2) = cam_T(2, 3);
  camera_r_m_ = cam_T.block<3, 3>(0, 0);
}

void ColorDetector::depthColorImgCallback(const sensor_msgs::ImageConstPtr& depth_align_msg, const sensor_msgs::ImageConstPtr& color_msg)
{
	/* get depth image */
	cv_bridge::CvImagePtr depth_cv_ptr;
	depth_cv_ptr = cv_bridge::toCvCopy(depth_align_msg, depth_align_msg->encoding);
	depth_cv_ptr->image.copyTo(depth_aligned_img_);

	/* get color image */
	cv_bridge::CvImagePtr color_cv_ptr;
	color_cv_ptr = cv_bridge::toCvCopy(color_msg, color_msg->encoding);
	color_cv_ptr->image.copyTo(color_img_);

	// cv::cvtColor(color_img_, color_img_, CV_BGR2RGB);
	//convert frame from BGR to HSV colorspace
    cv::cvtColor(color_img_, HSV_img_, CV_BGR2HSV);
    cv::inRange(HSV_img_, cv::Scalar(H_MIN_, S_MIN_, V_MIN_), cv::Scalar(H_MAX_, S_MAX_, V_MAX_), threshold_);
    if(USE_MORPHOPS_)
      morphOps(threshold_);
    if(USE_TRACK_)
      trackFilteredObject(x,y,z,threshold_, color_img_, depth_aligned_img_, is_detected);
	
	if(IS_SHOW_) {
		cv::imshow("raw color img", color_img_);
		cv::waitKey(50);
	}

	// if(IS_SHOW_) {
	// 	cv::imshow("hsv img", HSV_img_);
	// 	cv::waitKey(50);
	// }

	if(IS_SHOW_) {
		cv::imshow("threshold img", threshold_);
		cv::waitKey(50);
	}

	if (ekf_3d.ifSetup()){
		showPredict();
		pubVisualDynamic();
	}
}

void ColorDetector::dynamicCallback(const geometry_msgs::PoseStamped& msg){
	if (simulator_first_){
		float ts = 1.0 / 15;
		MatrixXf A(6,6);
		A << 1, 0, 0, ts, 0, 0,
			0, 1, 0, 0, ts, 0,
			0, 0, 1, 0, 0, ts,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1;
		MatrixXf B(6,1);
		B << 0, 0, 0, 0, 0, 0;

		MatrixXf H(3,6);
		H << 1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0;

		MatrixXf P = MatrixXf::Identity(6, 6);
		MatrixXf Q = MatrixXf::Identity(6, 6);
		MatrixXf R = MatrixXf::Identity(3, 3);
		for(int i = 0; i < 3; i ++){
			P(i,i) = 1.0;
			Q(i,i) = 1.0;
			R(i,i) = 0.05;
		}

		for(int i = 3; i < 6; i++){
			P(i,i) = 1.0;
			Q(i,i) = 1.0;
		}   

		MatrixXf states(6,1);
		states << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 0, 0, 0;
		ros::Time global_start_time = ros::Time::now();
		ekf_3d.setup( states,  A,  B,  H,  P,  Q,  R, global_start_time);

		simulator_first_ = false;
	} else {
		MatrixXf z(3,1);
		z << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
		MatrixXf u(1,1);
		u << 0;
		ros::Time global_start_time = ros::Time::now();
		ekf_3d.estimate(z, u, global_start_time);
	}

	if (ekf_3d.ifSetup()){
		showPredict();
		pubVisualDynamic();
	}
}

void ColorDetector::morphOps(cv::Mat &thresh)
{
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	cv::Mat erodeElement = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(8,8));
	cv::erode(thresh,thresh,erodeElement);
	cv::erode(thresh,thresh,erodeElement);
	cv::dilate(thresh,thresh,dilateElement);
	cv::dilate(thresh,thresh,dilateElement);
}
void on_trackbar( int, void* ){}

void ColorDetector::createTrackbars(){
	ROS_WARN("createTrackbars start");
	std::string trackbarWindowName = "Trackbars";
	//create window for trackbars
	cv::namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN_);
	sprintf( TrackbarName, "H_MAX", H_MAX_);
	sprintf( TrackbarName, "S_MIN", S_MIN_);
	sprintf( TrackbarName, "S_MAX", S_MAX_);
	sprintf( TrackbarName, "V_MIN", V_MIN_);
	sprintf( TrackbarName, "V_MAX", V_MAX_);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->
	cv::createTrackbar( "H_MIN", trackbarWindowName, &H_MIN_, 255, on_trackbar );
	cv::createTrackbar( "H_MAX", trackbarWindowName, &H_MAX_, 255, on_trackbar );
	cv::createTrackbar( "S_MIN", trackbarWindowName, &S_MIN_, 255, on_trackbar );
	cv::createTrackbar( "S_MAX", trackbarWindowName, &S_MAX_, 255, on_trackbar );
	cv::createTrackbar( "V_MIN", trackbarWindowName, &V_MIN_, 255, on_trackbar );
	cv::createTrackbar( "V_MAX", trackbarWindowName, &V_MAX_, 255, on_trackbar );
	cv::createTrackbar( "MAX_NUM_OBJECTS", trackbarWindowName, &MAX_NUM_OBJECTS_, 5, on_trackbar );
	cv::createTrackbar( "MIN_OBJECT_AREA", trackbarWindowName, &MIN_OBJECT_AREA_, IMG_WIDTH_*IMG_HEIGHT_*0.05, on_trackbar );
	cv::createTrackbar( "MAX_OBJECT_AREA", trackbarWindowName, &MAX_OBJECT_AREA_, IMG_WIDTH_*IMG_HEIGHT_, on_trackbar );
	cv::waitKey(1);
	ROS_WARN("createTrackbars end");
}

void ColorDetector::checkParams()
{
	ROS_WARN("checkParams() end!!!!");
	std::cout << "img_width_ = " << IMG_WIDTH_ << std::endl;
	std::cout << "H_MIN = " << H_MIN_ << std::endl;
	std::cout << "IS_SHOW = " << IS_SHOW_ << std::endl;
	
	ROS_WARN("checkParams() end!!!!");

}

void ColorDetector::trackFilteredObject(int &x, int &y, float &z, cv::Mat threshold, cv::Mat &cameraFeed, cv::Mat &depth_img, bool &isDetect){
	cv::Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	cv::findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find out filtered object
	double refArea = 0;
	bool objectFound = false;
	int numObjects = hierarchy.size();

	// std::cout << "hierarchy.size = " << hierarchy.size() << std::endl;
	// for(int i = 0; i < hierarchy.size(); i++) {
	// 	std::cout << i << ": (" << hierarchy[i][0] << ", " << hierarchy[i][1] << ", " << hierarchy[i][2] << ", " << hierarchy[i][3] << ")" << std::endl;
	// }
	
	if (numObjects > 0) {
		std::vector<cv::Moments> moment_vec;
		cv::Mat dst;
		for (int index = 0; index >= 0; index = hierarchy[index][0])
		{
			cameraFeed.copyTo(dst, temp);
			// draw every contours
			cv::drawContours(dst, contours, index, cv::Scalar(0,255,0));
			// if(IS_SHOW_) {
			// 	cv::imshow("contours", dst);
			// 	cv::waitKey(50);
			// }
			cv::Moments moment = cv::moments((cv::Mat)contours[index]);			
			moment_vec.push_back(moment);
		}
		sort(moment_vec.begin(), moment_vec.end(), cmp_max);

		for (int index = 0; index < moment_vec.size() && index < MAX_NUM_OBJECTS_; index++) {
			double area = moment_vec[index].m00;
			objectFound = false;
			if(area > MIN_OBJECT_AREA_ && area < MAX_OBJECT_AREA_){
				x = moment_vec[index].m10/area;
				y = moment_vec[index].m01/area;
				objectFound = true;
			}
			//let user know you found an object
	      	if(objectFound == true){
				Eigen::Vector2i p(x, y);
				float depth, heading;
				pixel2Heading(depth_aligned_img_, p, heading, depth);
	        	z = depth;
				cv::putText(cameraFeed,"Tracking Object", cv::Point(0,50),2,1, cv::Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x,y,depth,cameraFeed);
				break;
			}
	      	else {
	        	z = -10;
	      	}
		}
	}
	else {
		// putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER", cv::Point(0,50), 1, 2, cv::Scalar(0,0,255), 2);
	}
		
	// kalman_filter
	if(objectFound){
		using namespace std;
		x_3d = (x - CX_) / FX_ * z;
		y_3d = (y - CY_) / FY_ * z;
		z_3d = z;

		Eigen::Vector3d local_3d;
		local_3d << x_3d, y_3d, z_3d;

		Eigen::Vector3d global_3d;
		Eigen::Matrix3f R;
		// R << 1, 0, 0,
		// 	0, 0, 1,
		// 	0, -1, 0;
		// R << 0, 0, 1,
		// 	-1, 0, 0,
		// 	0, -1, 0;
		// global_3d = R * local_3d + t_;
		global_3d = camera_r_m_ * local_3d + camera_pos_;

		if(object_first){
			float ts = 1.0 / 15;
			MatrixXf A(6,6);
			A << 1, 0, 0, ts, 0, 0,
				0, 1, 0, 0, ts, 0,
				0, 0, 1, 0, 0, ts,
				0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1;
			MatrixXf B(6,1);
			B << 0, 0, 0, 0, 0, 0;

			MatrixXf H(3,6);
			H << 1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0;

			MatrixXf P = MatrixXf::Identity(6, 6);
			MatrixXf Q = MatrixXf::Identity(6, 6);
			MatrixXf R = MatrixXf::Identity(3, 3);

			for(int i = 0; i < 3; i ++){
				P(i,i) = 1.0;
				Q(i,i) = 1.0;
				R(i,i) = 0.05;
			}

			for(int i = 3; i < 6; i++){
				P(i,i) = 1.0;
				Q(i,i) = 1.0;
			}   

			MatrixXf states(6,1);
			states << global_3d.x(), global_3d.y(), global_3d.z(), 0, 0, 0;
			ros::Time global_start_time = ros::Time::now();
			ekf_3d.setup( states,  A,  B,  H,  P,  Q,  R, global_start_time);

			object_first = false;

		}else{
			MatrixXf z(3,1);
			z << global_3d.x(), global_3d.y(), global_3d.z();
			MatrixXf u(1,1);
			u << 0;
			ros::Time global_start_time = ros::Time::now();
			ekf_3d.estimate(z, u, global_start_time);
		}
	}else{
		object_first = true;
	}

  isDetect = objectFound;
}


void ColorDetector::pixel2Point(cv::Mat& depth_img, Eigen::Vector2i& pixel, float* upoint, float& depth)
{

  uint16_t *row_ptr;
  row_ptr = depth_img.ptr<uint16_t>(pixel(1));
  depth = (*(row_ptr+pixel(0))) / 1000.0;

  upoint[0] = depth * (pixel(0) - CX_) / FX_;
  upoint[1] = depth * (pixel(1) - CY_) / FY_;
  upoint[2] = depth; 
}

bool ColorDetector::pixel2Heading(cv::Mat& depth_img, Eigen::Vector2i& pixel, float& heading, float& depth)
{
	float upoint[3];
	pixel2Point(depth_img, pixel, upoint, depth);

	if (abs(upoint[2]) < EPSILON) {
		return false;
	}
	else {
		if (upoint[0] > 0)
			heading = atan2f(upoint[0], upoint[2]);
		else
			heading = atan2f(upoint[0], upoint[2]) + PI * 2;
		return true;
	}
}

void ColorDetector::pubVisualDynamic(){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	geometry_msgs::Point start, end;
	start.x = ekf_3d.states(0);
	start.y = ekf_3d.states(1);
	start.z = ekf_3d.states(2);
	marker.points.push_back(start);
	end.x = ekf_3d.states(0) + 2.0 * ekf_3d.states(3);
	end.y = ekf_3d.states(1) + 2.0 * ekf_3d.states(4);
	end.z = ekf_3d.states(2) + 2.0 * ekf_3d.states(5);
	marker.points.push_back(end);

	// cout << "START" << endl;
	// cout << ekf_3d.states.block<3,1>(0,0) << endl;
	// cout << "END" << endl;
	// cout << end.x << end.y << end.z << endl;

	arrow_pub_.publish( marker );
}

void ColorDetector::showPredict(){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	// predict
	geometry_msgs::Point pt;
	double T  = 5.0;
	double dt = 0.1;
	int num   = int (T / dt);
	ros::Time t0 = ros::Time::now();
	for (int i=0; i<num; i++){
		Eigen::Vector3d pos;
		double t = t0.toSec() + dt * i;
		pos = ekf_3d.evaluateConstVel(t);
		pt.x = pos(0);
		pt.y = pos(1);
		pt.z = pos(2);
		marker.points.push_back(pt);
	}
	predict_show_pub_.publish( marker );
}

Eigen::Vector3d ColorDetector::evaluateEkf3DVel(double t){
	Eigen::Vector3d pos;
	pos = ekf_3d.evaluateConstVel(t);
	return pos;
}

bool ColorDetector::ifDetect(){
	return ekf_3d.ifSetup();
}
