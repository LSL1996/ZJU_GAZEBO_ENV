#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "string"
#include <sstream>
#include <algorithm>

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

// synchronize topic
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// OpenCV and Eigen
#include <Eigen/Eigen>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// EKF
#include "kalman_filter.h"


class ColorDetector
{
public:
	ColorDetector(){};
	~ColorDetector(){};

	void init(ros::NodeHandle& nh);
	Eigen::Vector3d evaluateEkf3DVel(double t);
	bool ifDetect();

	typedef std::shared_ptr<ColorDetector> Ptr;

private:
	void readParameters(ros::NodeHandle& nh);
	void registerSubAndPub(ros::NodeHandle& nh);
	
	void createTrackbars();
	void drawObject(int x, int y, float depth, cv::Mat &frame);
	void pixel2Point(cv::Mat& depth_img, Eigen::Vector2i& pixel, float* upoint, float& depth);
	bool pixel2Heading(cv::Mat& depth_img, Eigen::Vector2i& pixel, float& heading, float& depth);

	void morphOps(cv::Mat &thresh); 
	void trackFilteredObject(int &x, int &y, float &z, cv::Mat threshold, cv::Mat &cameraFeed, cv::Mat &depth_img, bool &isDetect);

	void pubVisualDynamic();

	// callback
	void depthColorImgCallback(const sensor_msgs::ImageConstPtr& depth_align_msg, const sensor_msgs::ImageConstPtr& color_msg);
	void dynamicCallback(const geometry_msgs::PoseStamped& msg);
	void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
	void extrinsicCallback(const nav_msgs::OdometryConstPtr &odom);

	// for debug
	void checkParams();
	void showPredict();

	// ROS topic subscriber
	// ros::Subscriber depth_aligned_img_sub_, color_img_sub_;

	// Time sync
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyDepthColorImage;
 	typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyDepthColorImage>> SynchronizerDepthColorImage;
 	SynchronizerDepthColorImage sync_depth_color_img_;

	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> color_img_sub_;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_aligned_img_sub_;
	ros::Subscriber odom_sub_, extrinsic_sub_, depth_sub_, color_sub_;
	ros::Subscriber dynamic_sub_;

	// ROS topic publisher
	ros::Publisher obj_vel_pub_;
	ros::Publisher arrow_pub_;
	ros::Publisher predict_show_pub_;


	// parameters
	//	camera params
	int IMG_WIDTH_, IMG_HEIGHT_;
	float FX_,FY_,CX_,CY_;
	//	hsv params
	int H_MIN_, H_MAX_, S_MIN_, S_MAX_, V_MIN_, V_MAX_;
	int MAX_NUM_OBJECTS_, MIN_OBJECT_AREA_, MAX_OBJECT_AREA_;
	bool USE_MORPHOPS_, USE_TRACKBAR_, USE_TRACK_;
	// 	debug params
	bool IS_SHOW_;
	bool is_simulator_;
	bool simulator_first_ = true;

	// Img member
	cv::Mat depth_aligned_img_, color_img_, HSV_img_, threshold_;

	bool is_detected = false;
	float depth = -1;
	int x = 0, y = 0;
	float z = -1;

	float x_3d = 0.f, y_3d = 0.f, z_3d = 0.f;
	

	kalman_filter ekf_3d;

	Eigen::Matrix3f R_ = Eigen::Matrix3f::Identity();
	Eigen::Vector3f t_ = Eigen::Vector3f::Zero();
	Eigen::Matrix4d cam2body_;
	Eigen::Vector3d camera_pos_ = Eigen::Vector3d::Zero();
	Eigen::Matrix3d camera_r_m_ = Eigen::Matrix3d::Identity();

	bool object_first = true;

};