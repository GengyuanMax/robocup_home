/*
 * tracking.h
 *
 *  Created on: 07-2019
 *      Author: luz.martinez@tum.de
 */

#ifndef TRACKING_H_
#define TRACKING_H_

// C, C++
#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <map>
#include <list>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
 #include <ros/package.h>
// #include <tf/transform_listener.h>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

// srvs and msgs
#include <rhome_srvs/Onoff.h>

namespace rhome_perception {



using std::string;

class Tracking  {


private:
	 ros::NodeHandle priv;
	std::string _name;
	bool _enabled;

	// - - - - - Parameters - - - - - - -
	std::string _base_frame;
	std::string _rgbd_frame;
	std::string _rgb_frame;
	std::string _depth_frame;
	std::string _rgb_topic;
	std::string _depth_topic;

	sensor_msgs::ImageConstPtr rgb_in;
	sensor_msgs::ImageConstPtr depth_in;
	std::string image_frameid;

	cv::Mat ImageIn, DepthIn;
	std::string _algorithm;
	std::vector<std::string> tracking_alg;
	cv::Ptr<cv::Tracker> tracker;
	cv::Rect2d roi_bbox;

	// publishers

	//Subscribers 
	ros::Subscriber _subs_rgb;
	ros::Subscriber _subs_depth;

	// services
    ros::ServiceServer _active_server;
    ros::ServiceServer _set_server;

	// Listeners
	ros::Subscriber _depth_sub;


public:
	bool _is_on;
	bool ready_rgb;
	bool ready_depth;
	bool roi_ready;

	Tracking(std::string name);
	virtual ~Tracking();

	void run();


private:

	// - - - - - - S u b s c r i b e r   C a l l b a c k s  - - - - - - - - -
	void _process_depth(const sensor_msgs::ImageConstPtr& img);
	void _process_rgb(const sensor_msgs::ImageConstPtr& img);

	// - - - - - - - - - - - - S e r v i c e s - - - - - - - - -
	bool _active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) ;
	bool set_roi(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res);
	
	// - - - - -  Functions  - - - - - - - - - -

};

} /* namespace  */
#endif /* Tracking_H_ */



