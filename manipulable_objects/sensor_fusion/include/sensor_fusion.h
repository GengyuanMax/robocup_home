/*
 * sensor_fusion.h
 *
 *  Created on: 08-2019
 *      Author: luz.martinez@tum.de
 */

#ifndef SENSORFUSION_H_
#define SENSORFUSION_H_

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
#include <sensor_msgs/LaserScan.h>
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

class SensorFusion  {


private:
	 ros::NodeHandle priv;
	std::string _name;
	bool _enabled;

	// - - - - - Parameters - - - - - - -
	std::string _base_frame;
	std::string _sensor1_topic;
	std::string _sensor2_topic;

	sensor_msgs::ImageConstPtr rgb_in;
	sensor_msgs::ImageConstPtr depth_in;
	std::string image_frameid;

	cv::Mat ImageIn, DepthIn;
	std::string _algorithm;

	// publishers

	//Subscribers 
	ros::Subscriber _subs_sensor1;
	ros::Subscriber _subs_sensor2;

	// services
    ros::ServiceServer _active_server;

	// Listeners
	// ros::Subscriber _depth_sub;


public:
	bool _is_on;
	bool ready_sensor1;
	bool ready_sensor2;

	SensorFusion(std::string name);
	virtual ~SensorFusion();

	void run();


private:

	// - - - - - - S u b s c r i b e r   C a l l b a c k s  - - - - - - - - -
	void _process_sensor1(const sensor_msgs::LaserScanConstPtr& scan);
	void _process_sensor2(const sensor_msgs::LaserScanConstPtr& scan);
	
	// - - - - - - - - - - - - S e r v i c e s - - - - - - - - -
	bool _active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) ;

	
	// - - - - -  Functions  - - - - - - - - - -

};

} /* namespace  */
#endif /* SENSORFUSION_H_ */



