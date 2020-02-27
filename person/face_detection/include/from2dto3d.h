#ifndef FROM2DTO3D_H
#define FROM2DTO3D_H

// ROS
#include <ros/ros.h>
//#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// rhome_perception
#include "rhome_srvs/Position3d.h"

using namespace std;

namespace rhome_perception
{
    class From2Dto3D
    {
    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _priv_nh;

        // Service
        ros::ServiceServer _server;

        // Subscriber
        ros::Subscriber _sub;

        // tf transform listener
        //tf::TransformListener _listener;
        geometry_msgs::Point _msg;

        // Parameter
        string _topic_pc;
        string _service_pos3d;

        pcl::PointCloud<pcl::PointXYZ>::ConstPtr _cloud;
        //----------------Callack---------------
        void _processCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
        bool _returnPosition3d(rhome_srvs::Position3d::Request& req, rhome_srvs::Position3d::Response& res);

    public:
        From2Dto3D(ros::NodeHandle nh);
        ~From2Dto3D() {};


    }; // class From2Dto3D
} // namesace rhome_perception
#endif //FROM2DTO3D_H