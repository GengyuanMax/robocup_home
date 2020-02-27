/*
 * lrhome_3d.h
 *
 *  Created on: 07-2019
 *      Author: luz.martinez@tum.de
 */

#ifndef LRHOME3D_H_
#define LRHOME3D_H_

// C, C++
#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <map>
#include <list>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
 #include <ros/package.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


// pcl
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/vfh.h>
#include <pcl/features/fpfh.h>

// #include <pcl/surface/concave_hull.h>
// #include <pcl/surface/convex_hull.h>

// #include <object_pose_estimation/SQTypes.h>
// #include <object_pose_estimation/ObjectPoseEstimator.h>
// #include <sample_superquadric.h>


// srvs and msgs
#include <rhome_srvs/Onoff.h>
#include <geometry_msgs/Pose.h>

namespace rhome_perception {

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<pcl::Normal> PNormal;
typedef pcl::VFHSignature308 VFH;
typedef pcl::PointCloud<VFH> PVFH;
typedef pcl::FPFHSignature33 FPFH;
typedef pcl::PointCloud<pcl::FPFHSignature33> PFPFH;

using std::string;

class LRHOME3D  {


private:
	 ros::NodeHandle priv;
	std::string _name;
	bool _enabled;

	// - - - - - Parameters - - - - - - -
	std::string _base_frame;
	std::string _rgbd_frame;
	std::string _rgb_frame;
	std::string _depth_frame;
	std::string _rgb_topic;
	std::string _depth_topic;
	std::string _point_topic;
	std::string _imagePoint_topic;

	std::vector<std::string> descriptors;
	std::string _descriptor;

	ros::Time lasttime_transform;

	float _crop_width;
	float _crop_depth;
	float _crop_min_z;
	float _crop_max_z;
	// float _leaf_size;

	// publishers
	image_transport::Publisher _mask_pub;

	//Subscribers 
	ros::Subscriber _subs_rgb;
	ros::Subscriber _subs_depth;
	ros::Subscriber _subs_point;

	pcl::PCLPointCloud2::ConstPtr cloud_in;
	sensor_msgs::ImageConstPtr rgb_in;
	sensor_msgs::ImageConstPtr depth_in;
	std::string image_frameid;


	// services
    ros::ServiceServer _active_server;

	// Listeners
	ros::Subscriber _depth_sub;
	tf::TransformListener *_tf_listener;
	tf::TransformListener tf_listener;
	// pcl
	pcl::VoxelGrid<Point> sor;


public:
	bool _is_on;
	bool ready_rgb;
	bool ready_depth;
	bool ready_point;

	LRHOME3D();
	virtual ~LRHOME3D();


public:
	// - - - - -  Functions  - - - - - - - - - -
	void downsample_pointcloud(const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);
	bool transform_pointcloud(const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, std::string target_frame);

	void filter_voxel(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out, double _leaf_size = 0.008);
	void filter_passthrough(PointCloud cloud_in, PointCloud::Ptr cloud_out);
	void extract_planar(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> get_clusters(PointCloud::Ptr  cloud_in);
	PNormal::Ptr get_normals(PointCloud::Ptr cloud_in);
	PVFH::Ptr descriptor_vfh(PointCloud::Ptr cloud_in, PNormal::Ptr cloud_normals); 
	PFPFH::Ptr descriptor_fpfh(PointCloud::Ptr cloud_in, PNormal::Ptr cloud_normals);
};

} /* namespace  */
#endif /* LRHOME3D_H_ */



