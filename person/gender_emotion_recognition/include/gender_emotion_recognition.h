#ifndef GENDER_EMOTION_RECOGNITION_H
#define GENDER_EMOTION_RECOGNITION_H

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

#include "gender_classification/gender_classification.h"
#include "emotion_recognition/emotion_recognition.h"

using namespace std;

namespace rhome_perception
{

class GenderEmotionRecognition
{
public:
	GenderEmotionRecognition(string name);
	~GenderEmotionRecognition() {};

private:
	ros::NodeHandle _nh;
	string _name;

	ros::ServiceServer _server_emotion;
    ros::ServiceServer _server_gender;   

	string _graph_path_gender;
    string _graph_path_emotion;
	string _service_emotion;
    string _service_gender;

    GenderClassifier* _gender_model;
	EmotionRecognizer* _emotion_model;

	bool _emotion_service(rhome_srvs::Recognize::Request &req, rhome_srvs::Recognize::Response &res);
    bool _gender_service(rhome_srvs::Recognize::Request &req, rhome_srvs::Recognize::Response &res);

};


} // namespace rhome_perception


#endif // GENDER_EMOTION_RECOGNITION_H