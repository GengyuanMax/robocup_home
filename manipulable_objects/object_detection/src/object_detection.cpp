#include <object_detection.h>

namespace rhome_perception {

ObjectDetection::ObjectDetection(string name):_name(name){

    ros::NodeHandle priv("~"), nh;
    // - - - - - - - - l i s t e n e r s - - - - - - - - - - - -
   // _tf_listener = new tf::TransformListener(ros::Duration(10.0));

    // - - - - - - - p a r a m e t e r s - - - - - - - - - - -

    priv.param<std::string>("camera_info",_camera_info_depth,"/xtion2/depth/camera_info");
    priv.param<std::string>("rgbd_frame",_rgbd_frame,"xtion2_link");
    priv.param<std::string>("rgb_frame",_rgb_frame,"xtion2_rgb_optical_frame");
    priv.param<std::string>("depth_frame",_depth_frame,"xtion2_depth_optical_frame");
    priv.param<std::string>("point_topic",_point_topic,"/xtion2/depth/points");
    priv.param<std::string>("rgb_topic",_rgb_topic,"/xtion2/rgb/image_raw");
    priv.param<std::string>("depth_topic",_depth_topic,"/xtion2/depth/image_raw");
    priv.param<float>("crop_width", _crop_width, 2.0);
    priv.param<float>("crop_depth", _crop_depth, 2.0);
    priv.param<float>("crop_min_z", _crop_min_z, -2.0);
    priv.param<float>("crop_max_z", _crop_max_z, 2.0);
    priv.param<float>("leaf_size", _leaf_size, 0.008);
    priv.param<bool>("visualize", _visualize, true);

    // - - - - - - - - p u b l i s h e r s  - - - - - - - - - - - -



    // - - - - - - - - s e r v i c e s  - - - - - - - - - - - - -
    _active_server  = priv.advertiseService("active", &ObjectDetection::_active_service, this);
  //  _getobjects_server  = priv.advertiseService("get_objects", &ObjectDetection::getobjects_service, this);

}

ObjectDetection::~ObjectDetection() {
    // delete _tf_listener;
}

bool ObjectDetection::_active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_depthinf = priv.subscribe(_camera_info_depth, 1, &ObjectDetection::_process_camerainfo, this); 
            _subs_point = priv.subscribe(_point_topic, 1, &ObjectDetection::_process_point, this); 
            _subs_depth = priv.subscribe(_depth_topic, 1, &ObjectDetection::_process_depth, this); 
            _subs_rgb = priv.subscribe(_rgb_topic, 1, &ObjectDetection::_process_rgb, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+_imagePoint_topic+". . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_point.shutdown();
              _subs_depth.shutdown();
              _is_on = false;
              ready_point = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}

//bool ObjectDetection::getobjects_service(robmosys_srvs::objectinformation::Request  &req, robmosys_srvs::objectinformation::Response &res) {

//  if (req.object_shape.shape_name == "sphere")
//  {
//    for (int i = 0; i < sphere_list.size(); ++i){
//      geometry_msgs::PoseStamped p;
//      p.pose.position = sphere_list[i];
//      res.pose.push_back(p);
//    }
//  }
//  if (req.object_shape.shape_name == "cube")
//  {
//    for (int i = 0; i < cube_list.size(); ++i){
//      geometry_msgs::PoseStamped p;
//      p.pose.position = cube_list[i];
//      res.pose.push_back(p);
//    }
//  }
//  if (req.object_shape.shape_name == "cylinder")
//  {
//    for (int i = 0; i < cylinder_list.size(); ++i){
//      geometry_msgs::PoseStamped p;
//      p.pose.position = cylinder_list[i];
//      res.pose.push_back(p);
//    }
//  }

//return true;
//}


void ObjectDetection::_process_camerainfo(const sensor_msgs::CameraInfo &info){

    depth_cx=info.K[2];    depth_cy=info.K[5];
    depth_fx=info.K[0];    depth_fy=info.K[4];

}

void ObjectDetection::_process_point(const pcl::PCLPointCloud2::ConstPtr &point_cloud_in){

    if(!_is_on) return;
    cloud_in=point_cloud_in;
    ready_point = true;
}

 void ObjectDetection::_process_rgb(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    rgb_in=img;
    // image_frameid = img->header.frame_id;
    ready_rgb = true;
}

 void ObjectDetection::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    image_frameid = img->header.frame_id;
    ready_depth = true;
}

cv::Point ObjectDetection::point3d_to_point2d(geometry_msgs::Point pt_in){
  cv::Point pt_out;

    int x_min = 0 , x_max = depth_img_width, y_min =0, y_max = depth_img_height;
    int tx = 40;

    pt_out.x = pt_in.x*depth_fx/pt_in.z  + depth_cx + tx;
    pt_out.y = pt_in.y*depth_fy/pt_in.z  + depth_cy;


    if (pt_out.x < x_min )  pt_out.x = x_min;
    if (pt_out.x > x_max )  pt_out.x = x_max;
    if (pt_out.y < y_min )  pt_out.y = y_min;
    if (pt_out.y > y_max )  pt_out.y = y_max;

    return pt_out;
}

void ObjectDetection::filter_voxel(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out) {
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
	sor.filter(*cloud_out);

	ROS_INFO_STREAM("[filter_voxel] Initial points " << cloud_in->points.size() << " - Final points  " << cloud_out->points.size());
}

void ObjectDetection::filter_passthrough(PointCloud cloud_in, PointCloud::Ptr cloud_out) {
    PointCloud::Ptr cloud_ptr  = cloud_in.makeShared();
    pcl::PassThrough<pcl::PointXYZ> pass;

    // filter z
    pass.setInputCloud (cloud_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (_crop_min_z, _crop_max_z);
    pass.filter (*cloud_out);

    // //     // x: depth
    pass.setInputCloud (cloud_out);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-_crop_depth, _crop_depth);
    pass.filter (*cloud_out);
    // y: width
    pass.setInputCloud (cloud_out);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-_crop_width, _crop_width);
    pass.filter (*cloud_out);


	ROS_INFO_STREAM("[filter_passthrough] Initial points " << cloud_in.points.size() << " - Final points  " << cloud_out->points.size());

}

void ObjectDetection::extract_planar(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out) {

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudObj(new pcl::PointCloud<pcl::PointXYZ>());

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  // int i = 0, nr_points = static_cast<int>(cloud_after_z->points.size());

  // while (i < 5) // 0.3 * nr_points
  // {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      ROS_WARN("Could not estimate a planar model for the given dataset.");
      // break;
      return;
    }
    else
    {
      ROS_DEBUG_STREAM_THROTTLE(1, "Segmentation: " << inliers->indices.size());
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);

    // Get the points associated with objects
    extract.filter(*cloud_out);
    ROS_DEBUG_STREAM_THROTTLE(
        1, "PointCloud representing the planar component: " << cloud_out->points.size () << " data points." );

  //   ++i;
  // }

    ROS_INFO_STREAM("[extract_planar] Initial points " << cloud_in->points.size() << " - Final points  " << cloud_out->points.size());
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ObjectDetection::get_clusters(PointCloud::Ptr  cloud_in) {
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(15);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_in);
  ec.extract(cluster_indices);

  ROS_INFO_STREAM("[get_clusters] " << cluster_indices.size() << " detected  ");

  // Put clusters in a pointcloud vector
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objCluster(cluster_indices.size());
  for (std::size_t i = 0; i < cluster_indices.size(); ++i)
  {
    objCluster[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
    objCluster[i]->points.reserve(cluster_indices[i].indices.size());
    // double sx=0, sy=0, sz=0;
    for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin();
        pit != cluster_indices[i].indices.end(); ++pit)
    {
      objCluster[i]->points.push_back(cloud_in->points[*pit]);
      // sx+=cloud_in->points[*pit].x;
      // sy+=cloud_in->points[*pit].y;
      // sz+=cloud_in->points[*pit].z;
      //TODO Calcular pose
    }
    // int nsize=objCluster[i]->points.size();
    objCluster[i]->width = objCluster[i]->points.size();
    objCluster[i]->height = 1;
    objCluster[i]->is_dense = true;
    objCluster[i]->header =  cloud_in->header;
    //ROS_DEBUG_STREAM("Cluster " << i+1 << " with " << objCluster[i]->points.size());
   // ROS_INFO_STREAM("Pose " << sx/nsize << ", " << sy/nsize << ", "<< sz/nsize );
  }

  return objCluster;
}


cv::Rect ObjectDetection::get_rect(pcl::PointCloud<pcl::PointXYZ>::Ptr obj)
{
    double x_min=obj->points[0].x, y_min=obj->points[0].y, z_min=obj->points[0].z;
    double x_max=obj->points[0].x, y_max=obj->points[0].y, z_max=obj->points[0].z;

    for (int i=0; i < obj->points.size () ;i++)
    {
      if (obj->points[i].x < x_min ){
          x_min = obj->points[i].x;
          z_min = obj->points[i].z;
      }
       if (obj->points[i].y < y_min)
        y_min = obj->points[i].y;

      if (obj->points[i].x > x_max){
          x_max = obj->points[i].x;
          z_max = obj->points[i].z;
      }
      if (obj->points[i].y > y_max){
          y_max = obj->points[i].y;
      }
    }

    geometry_msgs::Point pt;
    bool sensor_zr = true;
    float tx = 0;
      if (sensor_zr)
      {
        tx =-0.088;
      }
    pt.x=x_min+tx;  pt.y=y_min;  pt.z=z_min;
    cv::Point pt_min = point3d_to_point2d(pt);
    pt.x=x_max+tx;  pt.y=y_max;  pt.z=z_max;
    cv::Point pt_max = point3d_to_point2d(pt);

    cv::Rect rect_out;
    rect_out.x = pt_max.x; rect_out.y = pt_max.y;
    rect_out.width = (pt_min.x-pt_max.x); rect_out.height = (pt_min.y-pt_max.y);

    return rect_out;
}


#include <sstream>

// - - - - -  RUN   - - - - - - - - - -
void ObjectDetection::run() {
    ROS_INFO_STREAM("[ObjectDetection] Processing new point cloud");

    ready_point = false;
    ready_depth = false;
    
    cv::Mat ImageIn, DepthIn;
    // ImageIn  = cv_bridge::toCvCopy((rgb_in), sensor_msgs::image_encodings::BGR8)->image;
    DepthIn = cv_bridge::toCvShare(depth_in)->image;
    depth_img_width = DepthIn.cols;
    depth_img_height = DepthIn.rows;

    // // Clouds
    PointCloud::Ptr cloud_helper      (new PointCloud);
    pcl::fromPCLPointCloud2(*cloud_in, *cloud_helper);
    PointCloud transformed_cloud, cloud_process; // transformed cloud
    PointCloud::Ptr filter1_cloud (new PointCloud), filter2_cloud (new PointCloud), objects_cloud (new PointCloud);

    // Step2: Crop table, robot and walls

    float initial_points = cloud_helper->size();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_helper, *cloud_helper, indices);
    float without_nan_points = cloud_helper->size();

    if (without_nan_points/initial_points > 0.3) // Checking valid point cloud
    {
        filter_passthrough(*cloud_helper, filter1_cloud);
        filter_voxel(filter1_cloud, filter2_cloud);

        // Step3: Planar extraction and get the objects in different clusters
        extract_planar(filter2_cloud, objects_cloud);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obj_clusters = get_clusters(objects_cloud);
        std::vector<cv::Rect> obj_rects;

        for (int i = 0; i < obj_clusters.size(); ++i)
        {
            if (obj_clusters[i]->size() < 300)   continue; // Filter of small clusters


            cv::Rect obj_rect = get_rect(obj_clusters[i]);
            if (abs(obj_rect.width) < 10 || abs(obj_rect.height) < 10 || abs(obj_rect.width) > depth_img_width*0.7 || abs(obj_rect.height) > depth_img_height*0.7 )
              continue;

            cv::rectangle(DepthIn, obj_rect, cv:: Scalar(0, 255, 255) ,1); 
        }
    }

  
  cv::imshow("Image",DepthIn);
  cv::waitKey(10);

}




} /* namespace rhome_perception */

int main(int argc, char** argv) {

    ros::init(argc, argv, "shape_recognition");
    ros::NodeHandle priv("~");

    boost::scoped_ptr<rhome_perception::ObjectDetection> node(
            new rhome_perception::ObjectDetection(ros::this_node::getName())
    );


    int _fps = 25;
    if(!priv.getParam("fps",_fps)) priv.setParam("fps",_fps);

    ros::Rate r(_fps);

    while(ros::ok()){
        if (node->_is_on && node->ready_point && node->ready_depth)
          node->run();
          
        // r.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Quitting ... \n");

    return 0;
}
