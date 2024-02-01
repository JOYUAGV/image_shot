#ifndef _IMAGE_SHOT_H
#define _IMAGE_SHOT_H

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <iomanip>
#include <fstream>
#include <iostream>
#include <sstream> // std::stringstream

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <sys/stat.h> 

class ImageShot {
public:
	/** \name Constructors/destructor @{ */
	/// Constructor
	ImageShot(const ros::NodeHandle& nh);
	/// Copy constructor
	ImageShot(const ImageShot& other) = delete;
	/// Copy assignment operator
	ImageShot& operator =
		(const ImageShot& other) = delete;
	/// Move constructor
	ImageShot(ImageShot&& other) = delete;
	/// Move assignment operator
	ImageShot& operator =
		(ImageShot&& other) = delete;
	/// Destructor
	virtual ~ImageShot();
	/** @} */

	/// Spin once
	void spin();
	/// Retrieves parameters
	void getParameters();
	/// NodeHandle
	ros::NodeHandle _nodeHandle;
private:
	/** \name Methods @{ */
	/// subscriber declare
	ros::Subscriber _rgbSubscriber;
	ros::Subscriber _depthSubscriber;
	/// topic name decalre
	std::string _rgbTopicName;
	std::string _depthTopicName;
	std::string _ImageShotTopicName;
	/// callback function declare
	void rgbCallback(const sensor_msgs::Image::ConstPtr& head);
	void depthCallback(const sensor_msgs::Image::ConstPtr& fix);
    void timerCallback(const ros::TimerEvent& event);
	/** @} */

	/** \name Variables @{ */
	/// _queueDepth
	int _queueDepth;
	double _control_frequency;
	/// Main Timer
	ros::Timer _timer;
	/// string 
	std::string _save_folder;
	std::string _data_set;
	std::string _rgb_prefix;
	std::string _depth_prefix;
	///int
	int _counter;
	/// double
	double _cv_rgb_stamp = 0;
	double _cv_depth_stamp = 0;
	double _delta_stamp = 0.001;
	/// CvImagePtr
	cv_bridge::CvImagePtr _cv_rgb_ptr;
	cv_bridge::CvImagePtr _cv_depth_ptr;
	/** @} */
};

#endif // _IMAGE_SHOT_H