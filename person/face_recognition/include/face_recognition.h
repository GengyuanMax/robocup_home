/**** 
Test based on database from AT&T Laboratories Cambridge
Source: https://www.cl.cam.ac.uk/research/dtg/attarchive/facedatabase.html
****/ 

#ifndef FACE_RECOGNITION_H
#define FACE_RECOGNITION_H

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

// Facenet
#include "Facenet.h"

using namespace std;
using namespace cv;

namespace rhome_perception
{

class FaceRecognition
{
public:
	FaceRecognition(string name);
	~FaceRecognition() {};

private:
	/*-------- DATA --------*/
	ros::NodeHandle _nh;
	string _name;

	ros::ServiceServer _server_recg;   

	ros::Publisher _pub_debug;

	string _database_path;
	string _graph_path;
	string _service_recg;

	Facenet* _facenet_model;

	/*-------- FUNCTIONS --------*/
	// Subscriber Callback
	void _process_roi(const rhome_msgs::FacePtr& face_msg);
	bool _recg_service(rhome_srvs::Recognize::Request &req, rhome_srvs::Recognize::Response &res);
};

} // namespace rhome_perception


#endif // FACE_RECOGNITION)H