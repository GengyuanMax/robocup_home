/*
 * instance_recognition.h
 *
 *  Created on: 07-2019
 *      Author: luz.martinez@tum.de
 */

#ifndef INSTANCERECOGNITION_H_
#define INSTANCERECOGNITION_H_

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
#include <boost/filesystem.hpp>

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
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
// #include "opencv2/nonfree/features2d.hpp"
// #include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"



// srvs and msgs
#include <rhome_srvs/Onoff.h>
#include <rhome_srvs/Queryobject.h>
#include <rhome_srvs/Getinformation.h>
#include <robmosys_srvs/objectinformation.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <rhome_msgs/Roi.h>
#include <sensor_msgs/CameraInfo.h>


#include "lbplibrary.hpp"


namespace rhome_perception {



using std::string;
using namespace boost::filesystem;


struct box_obj {
  cv::Point p1;
  cv::Point p2;
  cv::Point p3;
  cv::Point p4;
};


class InstanceRecognition  {


private:
	ros::NodeHandle priv;
	std::string _name;
	bool _enabled;
	std::vector<float> colorcamera_info;
	std::string db_path;
	bool train_succes;
	std::vector<cv::Mat> db_dscr;
	std::vector<std::string> db_label;
	std::vector<std::vector<cv::KeyPoint>> db_kp;
	std::vector<int> db_width;
	std::vector<int> db_heigth;


	// - - - - - Parameters - - - - - - -
	std::string _base_frame;
	std::string _rgbd_frame;
	std::string _rgb_frame;
	std::string _depth_frame;
	std::string _rgb_topic;
	std::string _depth_topic;
	std::string _depth_info_topic;

	sensor_msgs::ImageConstPtr rgb_in;
	sensor_msgs::ImageConstPtr depth_in;
	std::string image_frameid;

	cv::Mat ImageIn, DepthIn;
	std::string _algorithm;
	std::string _compare_hist;
	std::vector<std::string> recognition_alg;
	std::vector<std::string> compare_hist_methods;
	cv::Mat descriptors;
	
	//Detection Information
	std::vector<cv::Mat> detected_imgs;
	std::vector<geometry_msgs::PoseStamped> detected_pose;
	std::vector<rhome_msgs::Roi> detected_roi;
	std::vector<std::string> detected_label;
	cv::Mat depth_img;

	//LBP
	lbplibrary::LBP *lbp;

	//Features and Matching
	cv::Ptr<cv::Feature2D> detector;
	cv::Ptr<cv::Feature2D> extractor;
	cv::Ptr<cv::DescriptorMatcher> matcher;

	// publishers

	//Subscribers 
	ros::Subscriber _subs_rgb;
	ros::Subscriber _subs_depth;
	ros::Subscriber _subs_dinfo;
	
	// services
    ros::ServiceServer _active_server;
    ros::ServiceServer _getobjects_server;
    ros::ServiceServer _queryobjects_server;

	// Listeners
	ros::Subscriber _depth_sub;


public:
	bool _is_on;
	bool ready_rgb;
	bool ready_depth;
	bool roi_ready;

	InstanceRecognition(std::string name);
	virtual ~InstanceRecognition();

	void run();


private:

	// - - - - - - S u b s c r i b e r   C a l l b a c k s  - - - - - - - - -
	void _process_depth(const sensor_msgs::ImageConstPtr& img);
	void _process_rgb(const sensor_msgs::ImageConstPtr& img);
	void _process_depthinfo(const sensor_msgs::CameraInfo& info);

	// - - - - - - - - - - - - S e r v i c e s - - - - - - - - -
	bool _active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) ;
	bool getobjects_service(rhome_srvs::Getinformation::Request  &req, rhome_srvs::Getinformation::Response &res);
	bool queryobjects_service(rhome_srvs::Queryobject::Request  &req, rhome_srvs::Queryobject::Response &res);

	// - - - - -  Functions  - - - - - - - - - -
	bool load_database();
	cv::Mat get_histogram(cv::Mat1b const& image);
	cv::Mat extract_dscr(cv::Mat image_input, std::vector<cv::KeyPoint> &kp);
	std::vector<int>  matching_db(cv::Mat dscr_input, std::vector< std::vector<cv::DMatch> > &matches);
	void remove_duplicates(std::vector<box_obj> &objs, std::vector<cv::Rect> &objs_rects);
	void point_to_position(cv::Point point2d, geometry_msgs::Point &point3d);
	geometry_msgs::PoseStamped point2d_to_pose(cv::Point point_in);

};

} /* namespace  */
#endif /* INSTANCERECOGNITION_H_ */



