#include "gender_emotion_recognition.h"

namespace rhome_perception
{

GenderEmotionRecognition::GenderEmotionRecognition(string name) : _name(name)
{
	ros::NodeHandle _nh("~");

	// Parameter
	_nh.param<string>("service_emotion", _service_emotion, "emotion");
	_nh.param<string>("service_gender", _service_gender, "gender");
    _nh.param<string>("graph_path_emotion", _graph_path_emotion, "/home/rosmosys/keras2cpp/emotionnet.pb");   
    _nh.param<string>("graph_path_gender", _graph_path_gender, "/home/rosmosys/keras2cpp/gendernet.pb"); 
	
	// Initialize models
    cout << "Establishing GenderClassifier" << endl;
    _gender_model = new GenderClassifier(_graph_path_gender);

    cout << "Establishing EmotionRecognizer" << endl;
	_emotion_model = new EmotionRecognizer(_graph_path_emotion);
    
    _server_emotion = _nh.advertiseService(_service_emotion, &GenderEmotionRecognition::_emotion_service, this);
    _server_gender = _nh.advertiseService(_service_gender, &GenderEmotionRecognition::_gender_service, this);
}


bool GenderEmotionRecognition::_emotion_service(rhome_srvs::Recognize::Request &req, rhome_srvs::Recognize::Response &res)
{
	cv::Mat image = cv_bridge::toCvCopy(req.face, "bgr8")->image;
	res.name = _emotion_model->run(image);
}

bool GenderEmotionRecognition::_gender_service(rhome_srvs::Recognize::Request &req, rhome_srvs::Recognize::Response &res)
{
	cv::Mat image = cv_bridge::toCvCopy(req.face, "bgr8")->image;
	res.name = _gender_model->run(image);
}




} // namespace rhome_perception


// init the nodes
int main(int argc, char** argv)
{
	ros::init(argc, argv, "gender_emotion_recognition");
	ros::NodeHandle priv("~");

	ROS_INFO_STREAM("Established node gender_emotion_recognition");

	boost::scoped_ptr<rhome_perception::GenderEmotionRecognition> node(
			new rhome_perception::GenderEmotionRecognition(ros::this_node::getName())
	);
	ROS_INFO_STREAM("Established gender emotion recognition");

	int _fps = 25;
	if(!priv.getParam("fps",_fps)) priv.setParam("fps",_fps);

	ros::Rate r(_fps);

	ROS_INFO_STREAM("Starting");

	while (ros::ok())
	{
		ros::spinOnce();
	}
	
	ROS_INFO("Quitting ... \n");

	return 0;
}