// #include "../include/image_shot/ImageShot.h"
#include "image_shot/ImageShot.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_shot_node");
	ros::NodeHandle nh;
	try 
	{
		ImageShot node(nh);
		node.spin();
	}
	catch (const std::exception& e) 
	{
		ROS_ERROR_STREAM("Exception: " << e.what());
		return 1;
	}
	catch (...) 
	{
		ROS_ERROR_STREAM("Unknown Exception");
		return 1;
	}
	return 0;
}
