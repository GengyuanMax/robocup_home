#include "face_recognition.h"

namespace rhome_perception
{

FaceRecognition::FaceRecognition(string name) : _name(name)
{
	ros::NodeHandle _nh("~");

	// Parameter
	_nh.param<string>("database_path", _database_path, "/home/rosmosys/catkin_ws/src/rhome_perception/person/face_recognition/data/database.csv");
	_nh.param<string>("graph_path", _graph_path, "/home/rosmosys/catkin_ws/src/rhome_perception/person/face_recognition/model/20180402-114759/20180402-114759.pb");
	_nh.param<string>("service_recg", _service_recg, "recognize");

	_server_recg = _nh.advertiseService(_service_recg, &FaceRecognition::_recg_service, this);
	
	// Initialize face recognizer
	_facenet_model = new Facenet(_graph_path, _database_path);

	_pub_debug = _nh.advertise<sensor_msgs::Image>("debug", 1000);

}




bool FaceRecognition::_recg_service(rhome_srvs::Recognize::Request &req, rhome_srvs::Recognize::Response &res)
{
	//res.name = model->predict_label(cv_bridge::toCvShare(iter, "bgr8")->image);
	_pub_debug.publish(req.face);
	cv::Mat image = cv_bridge::toCvCopy(req.face, "bgr8")->image;
	res.name = _facenet_model->recognize(image);
	
	return true;
}


} // namespace: rhome_perception



int main(int argc, char** argv)
{
	ros::init(argc, argv, "face_recognition");
	ros::NodeHandle priv("~");

	ROS_INFO_STREAM("Established node face_recognition");

	boost::scoped_ptr<rhome_perception::FaceRecognition> node(
			new rhome_perception::FaceRecognition(ros::this_node::getName())
	);
	ROS_INFO_STREAM("Established face recognition");

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