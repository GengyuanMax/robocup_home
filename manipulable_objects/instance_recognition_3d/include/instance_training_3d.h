/*
 * instance_training_3d.h
 *
 *  Created on: 07-2019
 *      Author: luz.martinez@tum.de
 */

#ifndef INSTANCETRAINING3D_H_
#define INSTANCETRAINING3D_H_

// C, C++
#include <iostream>
#include <string>
#include <sstream>
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
#include <pcl/io/pcd_io.h> 
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

#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>

#include <lrhome_3d.h>

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

typedef std::pair<std::string, std::vector<float> > vfh_model;

using std::string;

class InstanceTraining3D  {


private:
	 ros::NodeHandle priv;
	std::string _name;
	bool _enabled;
	std::string db_path;

	// - - - - - Parameters - - - - - - -
	std::string _base_frame;
	std::string _rgbd_frame;
	std::string _rgb_frame;
	std::string _depth_frame;
	std::string _rgb_topic;
	std::string _depth_topic;
	std::string _point_topic;
	std::string _imagePoint_topic;
	bool _save_pointclouds;
	
	std::vector<std::string> descriptors;
	std::string _descriptor;

	ros::Time lasttime_transform;

	float _crop_width;
	float _crop_depth;
	float _crop_min_z;
	float _crop_max_z;
	float _leaf_size;



	cv::Point pt;
	bool newCoords;
	bool stop_saving;
	LRHOME3D lib3d;
	int n_object;
	std::string name_object;

	std::vector<vfh_model> models;
	std::string kdtree_idx_file_name;
	std::string training_data_h5_file_name;
	std::string training_data_list_file_name;

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
	cv::Mat ImageIn;


	// services
    ros::ServiceServer _active_server;
    ros::ServiceServer _train_server;
    ros::ServiceServer _save_server;

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

	InstanceTraining3D(std::string name);
	virtual ~InstanceTraining3D();

	void run();
	static void mouse_callback(int  event, int  x, int  y, int  flag, void *param);
	void save_ptmouse (int  x, int  y);
private:

	// - - - - - - S u b s c r i b e r   C a l l b a c k s  - - - - - - - - -
	void _process_point(const pcl::PCLPointCloud2::ConstPtr &point_cloud_in);
	void _process_depth(const sensor_msgs::ImageConstPtr& img);
	void _process_rgb(const sensor_msgs::ImageConstPtr& img);

	// - - - - - - - - - - - - S e r v i c e s - - - - - - - - -
	bool _active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) ;
	bool _train_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res);
	bool _save_database(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res);

	// - - - - -  Functions  - - - - - - - - - -
	bool  point2d_in_cloud(cv::Point pt, PointCloud::Ptr cloud_in);
	bool save_data(cv::Mat image_in);
	bool save_cluster (PointCloud::Ptr cloud_in);
	bool cluster_model(PVFH cluster_vfh, vfh_model &vfh);

	// void downsample_pointcloud(const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);
	// bool transform_pointcloud(const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, std::string target_frame);

	// void filter_voxel(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out);
	// void filter_passthrough(PointCloud cloud_in, PointCloud::Ptr cloud_out);
	// void extract_planar(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out);
	// std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> get_clusters(PointCloud::Ptr  cloud_in);
	// PNormal::Ptr get_normals(PointCloud::Ptr cloud_in);
	// PVFH::Ptr descriptor_vfh(PointCloud::Ptr cloud_in, PNormal::Ptr cloud_normals); 
	// PFPFH::Ptr descriptor_fpfh(PointCloud::Ptr cloud_in, PNormal::Ptr cloud_normals);
};

} /* namespace  */
#endif /* INSTANCETRAINING3D_H_ */



