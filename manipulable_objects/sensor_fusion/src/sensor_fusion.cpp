#include <sensor_fusion.h>

namespace rhome_perception {

SensorFusion::SensorFusion(string name):_name(name){

    ros::NodeHandle priv("~"), nh;
    // - - - - - - - - l i s t e n e r s - - - - - - - - - - - -
    // - - - - - - - p a r a m e t e r s - - - - - - - - - - -

    priv.param<std::string>("base_frame",_base_frame,"base_link");
    // priv.param<std::string>("rgbd_frame",_rgbd_frame,"xtion2_link");
    // priv.param<std::string>("rgb_frame",_rgb_frame,"xtion2_rgb_optical_frame");
    // priv.param<std::string>("depth_frame",_depth_frame,"xtion2_depth_optical_frame");
    // priv.param<std::string>("rgb_topic",_rgb_topic,"/xtion2/rgb/image_raw");
    priv.param<std::string>("sensor1_topic",_sensor1_topic,"/lidar1/scan");
    priv.param<std::string>("sensor2_topic",_sensor2_topic,"/lidar2/scan");

    ready_sensor1 = false;
    ready_sensor2 = false;

    // - - - - - - - - p u b l i s h e r s  - - - - - - - - - - - -



    // - - - - - - - - s e r v i c e s  - - - - - - - - - - - - -
    _active_server  = priv.advertiseService("active", &SensorFusion::_active_service, this);
    // _set_server  = priv.advertiseService("set_object", &SensorFusion::set_roi, this);
}

SensorFusion::~SensorFusion() {
}

 bool SensorFusion::_active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_sensor1 = priv.subscribe<sensor_msgs::LaserScan>(_sensor1_topic, 1, &SensorFusion::_process_sensor1, this); 
            _subs_sensor2 = priv.subscribe<sensor_msgs::LaserScan>(_sensor2_topic, 1, &SensorFusion::_process_sensor2, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+_sensor1_topic+". . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_sensor1.shutdown();
              _is_on = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}


void SensorFusion::_process_sensor1(const sensor_msgs::LaserScanConstPtr& scan){
    ROS_INFO("%lf",scan.get()->header.stamp.toSec());
    ready_sensor1 = true;
}

void SensorFusion::_process_sensor2(const sensor_msgs::LaserScanConstPtr& scan){
    ROS_INFO("%lf",scan.get()->header.stamp.toSec());
    ready_sensor2 = true;
}




// - - - - -  RUN   - - - - - - - - - -
void SensorFusion::run() {

}




} /* namespace rhome_perception */

int main(int argc, char** argv) {

    ros::init(argc, argv, "SensorFusion");
    ros::NodeHandle priv("~");

    boost::scoped_ptr<rhome_perception::SensorFusion> node(
            new rhome_perception::SensorFusion(ros::this_node::getName())
    );


    int _fps = 25;
    if(!priv.getParam("fps",_fps)) priv.setParam("fps",_fps);

    ros::Rate r(_fps);

    while(ros::ok()){
        if (node->_is_on && (node->ready_sensor1 || node->ready_sensor2))
          node->run();
          
        // r.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Quitting ... \n");

    return 0;
}
