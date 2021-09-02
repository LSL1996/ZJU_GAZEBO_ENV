#include <ros/ros.h>
#include "color_detector.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "red_obj_detector");
	ros::NodeHandle nh("~");

	ColorDetector red_detector;
	red_detector.init(nh);
	
	ros::spin();
	return 0;
}