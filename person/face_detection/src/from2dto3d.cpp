#include "from2dto3d.h"

namespace rhome_perception
{
    From2Dto3D::From2Dto3D(ros::NodeHandle nh) : _nh(nh), _priv_nh("~")
    {
        // Initialize parameter
        _nh.param<std::string>("topic_pc", _topic_pc, "/camera/depth/points");
        _nh.param<std::string>("service_pos3d", _service_pos3d, "position3d");

        // Subsciber
        _sub = _nh.subscribe<sensor_msgs::PointCloud2>(_topic_pc, 100, &From2Dto3D::_processCloud, this);

        // Server
        _server = _nh.advertiseService(_service_pos3d, &From2Dto3D::_returnPosition3d, this);

    }

    void From2Dto3D::_processCloud(const sensor_msgs::PointCloud2ConstPtr &pc)
    {

        if ((pc->width*pc->height==0)) return;
    }

    bool From2Dto3D::_returnPosition3d(rhome_srvs::Position3d::Request &req, rhome_srvs::Position3d::Response &res)
    {
        int cx = (int) req.bb.x + req.bb.width/2;
        int cy = (int) req.bb.y + req.bb.height/2;
        int ox = req.bb.x;
        int oy = req.bb.y;

        // Nan checking
        if (_cloud->empty()) return false;

        int position = cy * _cloud->width + cx;
        float x = _cloud->points[position].x;
        float y = _cloud->points[position].y;
        float z = _cloud->points[position].z;

        while (isnan(x) || isnan(y) || isnan(z)) {
            position = position + 1;
            x = _cloud->points[position].x;
            y = _cloud->points[position].y;
            z = _cloud->points[position].z;
        }

        res.pos.x = x;
        res.pos.y = y;
        res.pos.z = z;
    }

} // namespace rhome_perception