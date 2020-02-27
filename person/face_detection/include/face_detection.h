#ifndef FACE_DETECION_H
#define FACE_DETECION_H

// C++ Library
#include <iostream>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// OpenCV Library
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <rhome_msgs/Face.h>
#include <rhome_srvs/Onoff.h>
#include <rhome_srvs/Recognize.h>

// MTCNN
#include "network.h"
#include "mtcnn.h"

namespace rhome_perception
{
 	
class FaceDetection
{
public:
	FaceDetection(std::string name);

	~FaceDetection() {};

private:
	/*-------- DATA --------*/
	ros::NodeHandle _nh;
	std::string _name;

	mtcnn* _mtcnn_detector;

	// parameter 
	std::string _topic_img;
	std::string _topic_face;
	std::string _topic_roi;
	std::string _service_recg;
	std::string _service_gender;
	std::string _service_emotion;
	std::string _filepath;

	// Subscriber & Publisher
	ros::Subscriber _sub_rgb;
	ros::Publisher _pub_face;
	ros::Publisher _pub_roi;
	ros::Publisher _pub_debug;

	// Client
	ros::ServiceClient _client_recg;
	ros::ServiceClient _client_gender;
	ros::ServiceClient _client_emotion;

	float _crop_width;
	float _crop_height;

	/*-------- FUNCTIONS --------*/

	// Subscriber Callback
	void process_rgb(const sensor_msgs::ImageConstPtr& img_msg);

	// Member functions
	// void from2dto3d();
	
};

} 



#endif //FACE_DETECTION_H