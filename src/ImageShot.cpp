// #include "../include/image_shot/ImageShot.h"
#include "image_shot/ImageShot.h"

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/
ImageShot::ImageShot(const ros::NodeHandle& nh) 
	: _nodeHandle(nh), _counter(0)
{
	getParameters();

	_rgbSubscriber = _nodeHandle.subscribe(
		_rgbTopicName, _queueDepth,
		&ImageShot::rgbCallback, this);

  _depthSubscriber = _nodeHandle.subscribe(
		_depthTopicName, _queueDepth,
		&ImageShot::depthCallback, this);

  std::string base_path = "/image_sets";
  _save_folder = ros::package::getPath("image_shot") + base_path + "/" + _data_set + "/";

  struct stat sb;
  if (stat(_save_folder.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
      // Folder exists
      std::cout << "Folder already exists: " << _save_folder << std::endl;
  } else {
      // Folder does not exist, create it
      if (mkdir(_save_folder.c_str(), 0777) == 0) {
          std::cout << "Folder created: " << _save_folder << std::endl;
      } else {
          std::cerr << "Error creating folder: " << _save_folder << std::endl;
      }
  }

	_timer = _nodeHandle.createTimer(ros::Duration(1.0/_control_frequency),//_control_frequency@控制频率
		&ImageShot::timerCallback, this);
}

ImageShot::~ImageShot() 
{

}

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

/******************************************************************************/
/*                                                                     
	@brief	回调函数
	@para	无
	@return 无
*/
/******************************************************************************/
void ImageShot::spin() 
{	
	ros::spin();
}

void ImageShot::rgbCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  _cv_rgb_stamp = msg->header.stamp.toSec();
  try {
      _cv_rgb_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv::imshow("RGB Image", _cv_rgb_ptr->image);
}

void ImageShot::depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  _cv_depth_stamp = msg->header.stamp.toSec();
  try {
      _cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv::imshow("Depth Image", _cv_depth_ptr->image);
}

void ImageShot::timerCallback(const ros::TimerEvent& event)
{
  int key = cv::waitKey(1);
  // ROS_INFO_STREAM("key: " << key);
  // ROS_INFO_STREAM("_cv_rgb_stamp:" << _cv_rgb_stamp);
  // ROS_INFO_STREAM("_cv_depth_stamp:" << _cv_depth_stamp);
  // ROS_INFO("dt(s):%.7f\t", _cv_rgb_stamp - _cv_depth_stamp);

  // if (key == 32) {  // 空格键
  if (key == 13) {  // 回车键
    if(fabs(_cv_rgb_stamp - _cv_depth_stamp) < _delta_stamp){ //确保rgb与depth图像为给定时间差下的两帧图像
      std::string rgb_path = _save_folder + _rgb_prefix + std::to_string(_counter) + ".png";
      cv::imwrite(rgb_path, _cv_rgb_ptr->image);
      ROS_INFO_STREAM("RGB Image saved: " << rgb_path);

      std::string depth_path = _save_folder + _depth_prefix + std::to_string(_counter) + ".png";
      cv::imwrite(depth_path, _cv_depth_ptr->image);
      ROS_INFO_STREAM("Depth Image saved: " << depth_path);

      _counter++;
    }
  }
}


/******************************************************************************/
/*                                                                     
	@brief	加载参数
	@para	无
	@return 无
*/
/******************************************************************************/
void ImageShot::getParameters() 
{
	std::string param_ns = "image_shot/";

	_nodeHandle.param<std::string>(param_ns + "rgbTopicName", 
		_rgbTopicName, "/camera/color/image_raw");
	
	_nodeHandle.param<std::string>(param_ns + "depthTopicName", 
		_depthTopicName, "/camera/depth/image_rect_raw");

  _nodeHandle.param<std::string>(param_ns + "rgb_prefix", 
		_rgb_prefix, "rgb_");

  _nodeHandle.param<std::string>(param_ns + "depth_prefix", 
		_depth_prefix, "depth_");

  _nodeHandle.param<std::string>(param_ns + "data_set", 
		_data_set, "saved_images");

	_nodeHandle.param<double>(param_ns + "delta_stamp", _delta_stamp, 0.005);
	_nodeHandle.param<int>(param_ns + "queue_depth", _queueDepth, 100);
	_nodeHandle.param<double>(param_ns + "control_frequency", _control_frequency, 20.0);
}
