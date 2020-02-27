#ifndef GESTURE_RECOGNITION_NODE_H
#define GESTURE_RECOGNITION_NODE_H

// C++ Library
#include <iostream>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// OpenCV Library
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/face/facerec.hpp"
#include "opencv2/imgproc.hpp"

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <rhome_srvs/Onoff.h>
#include <rhome_msgs/Face.h>
#include <rhome_srvs/Recognize.h>

#include "gesture_recognition.h"

using namespace std;

namespace rhome_perception
{

class GestureRecognition
{

private:
    /*-------- DATA --------*/
	ros::NodeHandle _nh;
	string _name;

    // parameter
	string _graph_path_det;
    string _graph_path_cls;
	string _topic_img;
    string _topic_gesture;

    // Subscriber & Publisher
	ros::Subscriber _sub_img;
	ros::Publisher _pub_gesture;

    GestureRecognizer* _gesture_model;

    /*-------- FUNCTION --------*/
    // Subscriber Callback
	void process_img(const sensor_msgs::ImageConstPtr& img_msg);

public:
	GestureRecognition(string name);
	~GestureRecognition() {};

};


} // namespace rhome_perception


#endif // GESTURE_RECOGNITION_NODE_H