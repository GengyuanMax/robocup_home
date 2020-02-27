#include "gesture_recognition_node.h"

namespace rhome_perception
{

GestureRecognition::GestureRecognition(string name) : _name(name)
{
	ros::NodeHandle _nh("~");

	// Parameter
    _nh.param<string>("graph_path_det", _graph_path_det, "/home/rosmosys/catkin_ws/src/rhome_perception/person/gesture_recognition/model/frozen_inference_graph.pb");   
    _nh.param<string>("graph_path_cls", _graph_path_cls, "/home/rosmosys/catkin_ws/src/rhome_perception/person/gesture_recognition/model/hand_poses_wGarbage_10.pb"); 
    _nh.param<std::string>("topic_img", _topic_img, "/camera/color/image_raw");
	_nh.param<std::string>("topic_gesture", _topic_gesture, "gesture");
	
	// Initialize models
    cout << "Establishing GestureRecognizer" << endl;
    _gesture_model = new GestureRecognizer(_graph_path_det, _graph_path_cls);

	// Subscriber
	_sub_img = _nh.subscribe(_topic_img, 1, &GestureRecognition::process_img, this);

	// Publisher
	_pub_gesture = _nh.advertise<sensor_msgs::Image>(_topic_gesture, 1000);

}

void GestureRecognition::process_img(const sensor_msgs::ImageConstPtr& img_msg)
{
    // Convert from msg to mat
	cv_bridge::CvImageConstPtr ptr_img;
	std::vector<cv::Rect> face_rect;

	ptr_img = cv_bridge::toCvShare(img_msg, "bgr8");
	cv::Mat img = ptr_img->image;

    vector<vector<double>> boxes;
    vector<string> labels;
    
    _gesture_model->run(img, boxes, labels);


    // Visualize
    for (auto iter = boxes.begin(); iter != boxes.end(); iter++) {
        
        int left = iter[0][0];
        int right = iter[0][1];
        int top = iter[0][2];
        int bottom = iter[0][3];
        cout << left << " " << right << " " << top << " " << bottom << endl;

        // cv::rectangle(img, cv::Rect(x, y, width, height), cv::Scalar(0, 0, 255));
        int n = iter - boxes.begin();
        cv::rectangle(img, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(n, n, 255), 3);
        cv::putText(img, labels[n], cv::Point(left, top), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(n, n, 255));
    }

	// Convert from mat to msg
	sensor_msgs::ImagePtr gesture_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    _pub_gesture.publish(gesture_msg);

}

} // namespace rhome_perception

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gesture_recognition");
	ros::NodeHandle priv("~");

	ROS_INFO_STREAM("Established node gesture_recognition");

	boost::scoped_ptr<rhome_perception::GestureRecognition> node(
			new rhome_perception::GestureRecognition(ros::this_node::getName())
	);
	ROS_INFO_STREAM("Established gesture recognition");

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