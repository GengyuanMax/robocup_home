#include <tracking.h>

namespace rhome_perception {

Tracking::Tracking(string name):_name(name){

    ros::NodeHandle priv("~"), nh;
    // - - - - - - - - l i s t e n e r s - - - - - - - - - - - -
    // - - - - - - - p a r a m e t e r s - - - - - - - - - - -

    priv.param<std::string>("base_frame",_base_frame,"base_link");
    priv.param<std::string>("rgbd_frame",_rgbd_frame,"xtion2_link");
    priv.param<std::string>("rgb_frame",_rgb_frame,"xtion2_rgb_optical_frame");
    priv.param<std::string>("depth_frame",_depth_frame,"xtion2_depth_optical_frame");
    priv.param<std::string>("rgb_topic",_rgb_topic,"/xtion2/rgb/image_raw");
    priv.param<std::string>("depth_topic",_depth_topic,"/xtion2/depth/image_raw");
    priv.param<std::string>("algorithm",_algorithm,"MIL");



    tracking_alg = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE"};//, "CSRT"??

    if (_algorithm == tracking_alg[0])       tracker = cv::TrackerBoosting::create();
    else if (_algorithm == tracking_alg[1])  tracker = cv::TrackerMIL::create();
    else if (_algorithm == tracking_alg[2])  tracker = cv::TrackerKCF::create();
    else if (_algorithm == tracking_alg[3])  tracker = cv::TrackerTLD::create();
    else if (_algorithm == tracking_alg[4])  tracker = cv::TrackerMedianFlow::create();
    else if (_algorithm == tracking_alg[5])  tracker = cv::TrackerGOTURN::create();
    else if (_algorithm == tracking_alg[6])  tracker = cv::TrackerMOSSE::create();
    else {
        tracker = cv::TrackerKCF::create();
        ROS_INFO_STREAM(" Using KCF algorithm");
    }
    ROS_INFO_STREAM(" Using "+_algorithm+" algorithm");
    // if (_algorithm == tracking_alg[7])
    //     tracker = cv::TrackerCSRT::create();
    roi_ready = false;
    ready_rgb = false;
    ready_depth = false;

    // - - - - - - - - p u b l i s h e r s  - - - - - - - - - - - -



    // - - - - - - - - s e r v i c e s  - - - - - - - - - - - - -
    _active_server  = priv.advertiseService("active", &Tracking::_active_service, this);
    _set_server  = priv.advertiseService("set_object", &Tracking::set_roi, this);
}

Tracking::~Tracking() {
}

 bool Tracking::_active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_depth = priv.subscribe(_depth_topic, 1, &Tracking::_process_depth, this); 
            _subs_rgb = priv.subscribe(_rgb_topic, 1, &Tracking::_process_rgb, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+_depth_topic+". . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_depth.shutdown();
              _is_on = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}


void Tracking::_process_rgb(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    rgb_in=img;
    // image_frameid = img->header.frame_id;
    ImageIn  = cv_bridge::toCvCopy((rgb_in), sensor_msgs::image_encodings::BGR8)->image;
    if (rgb_in->height * rgb_in->width > 0)
        ready_rgb = true;
}

void Tracking::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    image_frameid = img->header.frame_id;
    DepthIn = cv_bridge::toCvShare(depth_in)->image;
    if (depth_in->height * depth_in->width > 0)
        ready_depth = true;

}


 bool Tracking::set_roi(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res){//TODO change srvs
    if (!ready_rgb){
        ROS_ERROR_STREAM("RGB image not ready, please call the service /active first");
        return true;
    }

    //roi_bbox = cv::Rect2d (0, 0, 100, 100); 
    roi_bbox = selectROI(ImageIn);
    tracker->init(ImageIn, roi_bbox);
    roi_ready = true;

    return true;
}


// - - - - -  RUN   - - - - - - - - - -
void Tracking::run() {
    ready_rgb = false;
    ready_depth = false;
    
    
    if (roi_ready)
    {
        bool t_result = tracker->update(ImageIn, roi_bbox);
        if (t_result)
            rectangle(ImageIn, roi_bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );
    }
    
    cv::imshow("Tracking", ImageIn); 
    cv::waitKey(5);
}




} /* namespace rhome_perception */

int main(int argc, char** argv) {

    ros::init(argc, argv, "tracking");
    ros::NodeHandle priv("~");

    boost::scoped_ptr<rhome_perception::Tracking> node(
            new rhome_perception::Tracking(ros::this_node::getName())
    );


    int _fps = 25;
    if(!priv.getParam("fps",_fps)) priv.setParam("fps",_fps);

    ros::Rate r(_fps);

    while(ros::ok()){
        if (node->_is_on && node->ready_depth && node->ready_rgb)
          node->run();
          
        // r.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Quitting ... \n");

    return 0;
}
