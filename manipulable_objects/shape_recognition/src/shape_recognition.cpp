#include <shape_recognition.h>

namespace rhome_perception {

ShapeRecognition::ShapeRecognition(string name):_name(name){

    ros::NodeHandle priv("~"), nh;
    // - - - - - - - - l i s t e n e r s - - - - - - - - - - - -
    _tf_listener = new tf::TransformListener(ros::Duration(10.0));

    // - - - - - - - p a r a m e t e r s - - - - - - - - - - -

    priv.param<std::string>("base_frame",_base_frame,"base_link");
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
    _active_server  = priv.advertiseService("active", &ShapeRecognition::_active_service, this);
    _getobjects_server  = priv.advertiseService("get_objects", &ShapeRecognition::getobjects_service, this);

}

ShapeRecognition::~ShapeRecognition() {
    delete _tf_listener;
}

bool ShapeRecognition::_active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_point = priv.subscribe(_point_topic, 1, &ShapeRecognition::_process_point, this); 
            _subs_depth = priv.subscribe(_depth_topic, 1, &ShapeRecognition::_process_depth, this); 
            _subs_rgb = priv.subscribe(_rgb_topic, 1, &ShapeRecognition::_process_rgb, this); 
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

bool ShapeRecognition::getobjects_service(robmosys_srvs::objectinformation::Request  &req, robmosys_srvs::objectinformation::Response &res) {

  if (req.object_shape.shape_name == "sphere")
  {
    for (int i = 0; i < sphere_list.size(); ++i){
      geometry_msgs::PoseStamped p;
      p.pose.position = sphere_list[i];
      res.pose.push_back(p);
    }
  }
  if (req.object_shape.shape_name == "cube")
  {
    for (int i = 0; i < cube_list.size(); ++i){
      geometry_msgs::PoseStamped p;
      p.pose.position = cube_list[i];
      res.pose.push_back(p);
    }
  }
  if (req.object_shape.shape_name == "cylinder")
  {
    for (int i = 0; i < cylinder_list.size(); ++i){
      geometry_msgs::PoseStamped p;
      p.pose.position = cylinder_list[i];
      res.pose.push_back(p);
    }
  }

return true;
}


void ShapeRecognition::_process_point(const pcl::PCLPointCloud2::ConstPtr &point_cloud_in){

    if(!_is_on) return;
    cloud_in=point_cloud_in;
    ready_point = true;
}

 void ShapeRecognition::_process_rgb(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    rgb_in=img;
    // image_frameid = img->header.frame_id;
    ready_rgb = true;
}

 void ShapeRecognition::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    image_frameid = img->header.frame_id;
    ready_depth = true;
}

cv::Point ShapeRecognition::point3d_to_point2d(geometry_msgs::Point pt_in){
  cv::Point pt_out;

    //TODO take values of camera_info
    //TODO transform point of rgb to depth frame
    float cx=308.2192687988281,  cy = 245.59515380859375; //rgb intrinsic
    //float fx = 473.1015319824219, fy = 473.1015625;
    float fx =  618.5377197265625, fy = 618.5377807617188;
   
    int x_min = 0 , x_max = 640, y_min =0, y_max = 480;
    int tx = 40;

    pt_out.x = pt_in.x*fx/pt_in.z  + cx + tx;
    pt_out.y = pt_in.y*fy/pt_in.z  + cy;


    if (pt_out.x < x_min )  pt_out.x = x_min;
    if (pt_out.x > x_max )  pt_out.x = x_max;
    if (pt_out.y < y_min )  pt_out.y = y_min;
    if (pt_out.y > y_max )  pt_out.y = y_max;

    return pt_out;
}

void ShapeRecognition::filter_voxel(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out) {
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
	sor.filter(*cloud_out);

	ROS_INFO_STREAM("[filter_voxel] Initial points " << cloud_in->points.size() << " - Final points  " << cloud_out->points.size());
}

void ShapeRecognition::filter_passthrough(PointCloud cloud_in, PointCloud::Ptr cloud_out) {
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

void ShapeRecognition::extract_planar(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out) {

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

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ShapeRecognition::get_clusters(PointCloud::Ptr  cloud_in) {
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


cv::Rect ShapeRecognition::get_rect(pcl::PointCloud<pcl::PointXYZ>::Ptr obj)
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


int ShapeRecognition::quadratic_form_matcher(const ope::SQParameters& param, const std::string& frame_id = "camera_depth_optical_frame")
{
		  // using namespace Eigen;
	ROS_INFO("Pose detected x=%.3f, y=%.3f, z=%.3f]", param.px, param.py, -param.pz);
	float eps = 0.3;
	//quadraticSphereMatcher
	if (std::abs(param.e1 - 1.0) < eps && std::abs(param.e2 - 1.0) < eps)
  // if ( (param.e1 > 0.8)  &&  (param.e2 > 0.8) )
	{
		// Calc diameter from parameters
		double diameter = 2.0/3.0 * (param.a1 + param.a3 + param.a3);
		ROS_INFO("Sphere detected (diameter = %.3f)", diameter);
    ROS_INFO_STREAM(param);
		return 1;
	}
	//quadraticCuboidMatcher
	if (std::abs(param.e1 - 0.1) < eps && std::abs(param.e2 - 0.1) < eps)
	{
		// Calc dimensions from parameters
		double depth = 2 * param.a1, width = 2 * param.a2, height = 2 * param.a3;
		ROS_INFO("Cube detected [depth=%.3f, widh=%.3f, height=%.3f]", depth, width, height);
    ROS_INFO_STREAM(param);
		return 2;
	}

	//quadraticCylinderMatcher

	// Calc height and radius from parameters
	double height = 2 * param.a3;
	double radius = param.a1 + param.a2;
	ROS_INFO("Cylinder detected [height=%.3f, radius=%.3f]", height, radius);
  ROS_INFO_STREAM(param);

	return 3;

}


int ShapeRecognition::quadratic_matcher(pcl::PointCloud<pcl::PointXYZ>::Ptr& obj, const std::string& frame_id = "camera_depth_optical_frame")
{
  ope::SQFitting sqf;
  ope::SQParameters initParams, bestParams;
  sqf.estimateInitialParameters( *obj, initParams );

  int NUM_SCALES = 4;
  double errorThreshold = 2.0;
  double errorDiff = 1000.0;
  double errorValue = 0;
  double prevErrorValue = 0;

  for (int j = 0; j < NUM_SCALES && errorDiff >= errorThreshold; j++) {

    sqf.performShapeFitting( *obj, initParams, bestParams );

    errorValue = sqf.qualityOfFit(*obj, bestParams);
    errorDiff = abs(prevErrorValue - errorValue);
    prevErrorValue = errorValue;

    bestParams.copyTo(initParams);
   //initParams.min.iterations = settings.minIterations;   
    initParams.min.a1.type = ope::BOUNDED;
    initParams.min.a1.lowerBound = 0.020f;
    initParams.min.a1.upperBound = initParams.a1 + 0.015f;
    initParams.a1 = 0.05f;
  
    initParams.min.a2.type = ope::BOUNDED;
    initParams.min.a2.lowerBound = 0.020f;
    initParams.min.a2.upperBound = initParams.a2 + 0.015f;
    initParams.a2 = 0.05f;

    initParams.min.a3.type = ope::BOUNDED;
    initParams.min.a3.lowerBound = 0.020f;
    initParams.min.a3.upperBound = initParams.a3 + 0.015f;
    initParams.a3 = 0.05f;

    initParams.min.e1.type = ope::BOUNDED;
    initParams.min.e1.lowerBound = 0.1f;
    initParams.min.e1.upperBound = 1.0f;
    initParams.e1 = 1.0f;

    initParams.min.e2.type = ope::BOUNDED;
    initParams.min.e2.lowerBound = 0.1f;
    initParams.min.e2.upperBound = 1.0f;
    initParams.e2 = 1.0f;
  
    initParams.min.phi.type = ope::UNLIMITED;
    initParams.min.theta.type = ope::UNLIMITED;
    initParams.min.psi.type = ope::UNLIMITED;
    initParams.min.phi.value = 1.0f;
    initParams.min.theta.value = 1.0f;
    initParams.min.psi.value = 1.0f;
  
    initParams.min.px.type = ope::UNLIMITED;
    initParams.min.py.type = ope::UNLIMITED;
    initParams.min.pz.type = ope::UNLIMITED;
    
    // if (settings.allowTapering) {
      initParams.min.kx.type = ope::BOUNDED;
      initParams.min.kx.lowerBound = -0.25f;
      initParams.min.kx.upperBound = 0.25f;
      initParams.min.kx.value = 0.0f;
      initParams.kx = 1.0f;
    
      initParams.min.ky.type = ope::BOUNDED;
      initParams.min.ky.lowerBound = -0.25f;
      initParams.min.ky.upperBound = 0.25;
      initParams.min.ky.value = 0.0f;
      initParams.ky = 1.0f;

    // } else {
    //   initParams.min.kx.type = UNCHANGED;
    //   initParams.min.kx.value = 0.0f; 
    //   initParams.min.ky.type = UNCHANGED;
    //   initParams.min.ky.value = 0.0f;

    // }   

  }


	int id_form = quadratic_form_matcher(bestParams, frame_id);

  geometry_msgs::Point pt;
  pt.x=bestParams.px;  pt.y=bestParams.py;  pt.z=bestParams.pz;

if (id_form == 1) sphere_list.push_back(pt);
if (id_form == 2) cube_list.push_back(pt);
if (id_form == 3) cylinder_list.push_back(pt);


  // cv::Point center = point3d_to_point2d(pt);
  //cv::Rect rect;
  // if (id_form == 1) // Sphere
  // {
  //     double diameter = 2.0/3.0 * (bestParams.a1 + bestParams.a3 + bestParams.a3);
  //     rect.x = center.x- diameter/2;  rect.y = center.y- diameter/2;
  //     rect.width = diameter;  rect.height = diameter;
  // }
  // if (id_form == 2 || id_form == 3) // Cube or Cylinder
  // {
  //     double height = 2 * bestParams.a3;   double diameter = bestParams.a1 + bestParams.a2;//IT said radius...
  //     rect.x = center.x - diameter/2;  rect.y = center.y - height/2;
  //     rect.width = diameter;  rect.height = diameter;
  // }
	// Print parameters
	return id_form;
}


#include <sstream>

// - - - - -  RUN   - - - - - - - - - -
void ShapeRecognition::run() {
    ready_point = false;
    ready_depth = false;
    
    cv::Mat ImageIn, DepthIn;
    ImageIn  = cv_bridge::toCvCopy((rgb_in), sensor_msgs::image_encodings::BGR8)->image;
    DepthIn = cv_bridge::toCvShare(depth_in)->image;

    // // Clouds
    PointCloud::Ptr cloud_helper      (new PointCloud);
    pcl::fromPCLPointCloud2(*cloud_in, *cloud_helper);
    PointCloud transformed_cloud, cloud_process; // transformed cloud
    PointCloud::Ptr filter1_cloud (new PointCloud), filter2_cloud (new PointCloud), objects_cloud (new PointCloud);

    // Step2: Crop table, robot and walls
    filter_passthrough(*cloud_helper, filter1_cloud);
    filter_voxel(filter1_cloud, filter2_cloud);

    // Step3: Planar extraction and get the objects in different clusters
  extract_planar(filter2_cloud, objects_cloud);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obj_clusters = get_clusters(objects_cloud);

	// Step4: Shape recognition 
   sphere_list.clear();
   cube_list.clear();
   cylinder_list.clear();
	for (std::size_t i = 0; i < obj_clusters.size(); ++i)
	{
		if (obj_clusters[i]->size() < 13)
		{
			ROS_WARN("Cluster with less than 13 points!");
			continue;
		}
   // 
        std::stringstream ss;
     ss << i;

    string n_shape = ss.str();
    string text_shape="";
		int form_id = quadratic_matcher(obj_clusters[i]);
		if (form_id == 1){
      text_shape="Sphere";
			ROS_INFO_STREAM("Obj "<<i+1<<" : Sphere");
    }
		if (form_id == 2){
      text_shape="Box";
			ROS_INFO_STREAM("Obj "<<i+1<<" : Cube");
    }
    if (form_id == 3){
      text_shape="Cylinder";
      ROS_INFO_STREAM("Obj "<<i+1<<" : Cylinder");
    }

    cv::Rect rect= get_rect(obj_clusters[i]);
    //cv::putText(ImageIn, n_shape, cv::Point (rect.x+rect.width/2 -30 , rect.y ), 1, 1, cv::Scalar(255, 0, 0));
    cv::putText(ImageIn, text_shape, cv::Point (rect.x+rect.width/2 -30 , rect.y + 20), 1, 1, cv::Scalar(255, 0, 0));
    cv::rectangle(ImageIn, rect, cv::Scalar(255, 0, 0), 2, 8, 0);
	}
  
  cv::imshow("Image",ImageIn);
  cv::waitKey(10);

}




} /* namespace rhome_perception */

int main(int argc, char** argv) {

    ros::init(argc, argv, "shape_recognition");
    ros::NodeHandle priv("~");

    boost::scoped_ptr<rhome_perception::ShapeRecognition> node(
            new rhome_perception::ShapeRecognition(ros::this_node::getName())
    );


    int _fps = 25;
    if(!priv.getParam("fps",_fps)) priv.setParam("fps",_fps);

    ros::Rate r(_fps);

    while(ros::ok()){
        if (node->_is_on && node->ready_point && node->ready_depth && node->ready_rgb)
          node->run();
          
        // r.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Quitting ... \n");

    return 0;
}
