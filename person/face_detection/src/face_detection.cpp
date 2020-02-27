#include <from2dto3d.h>
#include "face_detection.h"

namespace rhome_perception
{


FaceDetection::FaceDetection(std::string name) : _name(name)
{
	ros::NodeHandle _nh("~");

	// Parameter
	_nh.param<std::string>("topic_img", _topic_img, "/camera/color/image_raw");
	_nh.param<std::string>("topic_face", _topic_face, "face");
	_nh.param<std::string>("topic_roi", _topic_roi, "roi");
	_nh.param<std::string>("service_recg", _service_recg, "/face_recognition/recognize");
	_nh.param<std::string>("service_gender", _service_gender, "/gender_emotion_recognition/gender");
	_nh.param<std::string>("service_emotion", _service_emotion, "/gender_emotion_recognition/emotion");
	_nh.param<std::string>("filepath", _filepath, "/home/rosmosys/MTCNN-light/");

	_nh.param<float>("crop_width", _crop_width, 100.0);
	_nh.param<float>("crop_height", _crop_height, 100.0);

	// mtcnn
	_mtcnn_detector = new mtcnn(480, 640, _filepath);

	// Subscriber
	_sub_rgb = _nh.subscribe(_topic_img, 1, &FaceDetection::process_rgb, this);

	// Publisher
	_pub_face = _nh.advertise<sensor_msgs::Image>(_topic_face, 1000);
	_pub_roi = _nh.advertise<rhome_msgs::Face>(_topic_roi, 1000);
	//_pub_debug = _nh.advertise<sensor_msgs::Image>("debug", 1000);

	// Client
	_client_recg = _nh.serviceClient<rhome_srvs::Recognize>(_service_recg);
	_client_gender = _nh.serviceClient<rhome_srvs::Recognize>(_service_gender);
	_client_emotion = _nh.serviceClient<rhome_srvs::Recognize>(_service_emotion);
	//_client_pos3d = _nh.serviceClient<rhome_srvs::Position3d>(_service_pos3d);



}


void FaceDetection::process_rgb(const sensor_msgs::ImageConstPtr& img_msg)
{
	// Convert from msg to mat
	cv_bridge::CvImageConstPtr ptr_img;
	std::vector<cv::Rect> face_rect;

	ptr_img = cv_bridge::toCvShare(img_msg, "bgr8");
	cv::Mat img = ptr_img->image;

	// Detect face in the frame
	_mtcnn_detector->findFace(img, face_rect);

	//rhome_msgs::FacePtr face_msg;
	rhome_msgs::FacePtr roi_msg;

	// Draw the boundingbox
	for (std::vector<cv::Rect>::iterator iter = face_rect.begin(); iter!=face_rect.end(); iter++)
	{
		cv::rectangle(img, *iter, cv::Scalar(0, 0, 2), 3, 8, 0);

		// call service 
		rhome_srvs::Recognize srv_recg, srv_gender, srv_emotion;
		string name, gender, emotion;

		cv::Mat roi = cv::Mat(img, cv::Rect(iter->x, iter->y, iter->width, iter->height));

 		cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi).toImageMsg(srv_recg.request.face);
		cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi).toImageMsg(srv_gender.request.face);
		cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi).toImageMsg(srv_emotion.request.face);

		if (_client_recg.call(srv_recg)) {
			cout << "detect person : " << srv_recg.response.name << endl;
			name = srv_recg.response.name;
		} else {
			ROS_ERROR_STREAM("Failed to call service Recognize");
		}

		if (_client_gender.call(srv_gender)) {
			gender = srv_gender.response.name;
			cout << "gender : " << gender << endl;
		} else {
			ROS_ERROR_STREAM("Failed to call service Gender");
		}

		if (_client_emotion.call(srv_emotion)) {
			emotion = srv_emotion.response.name;
			cout << "emotion : " << emotion << endl;
		} else {
			ROS_ERROR_STREAM("Failed to call service Emotion");
		}

		string info;
		info = name + " " + gender + " " + emotion;
		cv::putText(img, info, cv::Point(iter->x, iter->y), FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,2), 3, 8, 0);

	}

	// Convert from mat to msg
	sensor_msgs::ImagePtr face_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

	// Publish
	_pub_face.publish(face_msg);

}

} //namespace rhome_perception

int main(int argc, char** argv)
{
	ros::init(argc, argv, "face_detection");
	ros::NodeHandle priv("~");

	ROS_INFO_STREAM("Established node face_detection");

	boost::scoped_ptr<rhome_perception::FaceDetection> node(
			new rhome_perception::FaceDetection(ros::this_node::getName())
	);
	ROS_INFO_STREAM("Established face detection");

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