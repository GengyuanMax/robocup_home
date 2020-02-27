#include <instance_recognition_3d.h>

//TODO Threshold
//TODO Display 

namespace rhome_perception {

InstanceRecognition3D::InstanceRecognition3D(string name):_name(name){

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
    priv.param<std::string>("descriptor",_descriptor, "VFH");
    priv.param<std::string>("training_h5_file",training_data_h5_file_name,"training_data.h5");
    priv.param<std::string>("training_list_file",training_data_list_file_name,"training_data.list");
    priv.param<std::string>("kdtree_idx_file",kdtree_idx_file_name,"kdtree.idx");


    descriptors = {"VFH", "FPFH"};

    ROS_INFO_STREAM(" Using "+_descriptor+" algorithm");

	db_path = ros::package::getPath("perception_databases");

	if (!load_database())
	{
		/* code */
	}
    // - - - - - - - - p u b l i s h e r s  - - - - - - - - - - - -



    // - - - - - - - - s e r v i c e s  - - - - - - - - - - - - -
    _active_server  = priv.advertiseService("active", &InstanceRecognition3D::_active_service, this);

}

InstanceRecognition3D::~InstanceRecognition3D() {
    delete _tf_listener;
}

 bool InstanceRecognition3D::_active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_point = priv.subscribe(_point_topic, 1, &InstanceRecognition3D::_process_point, this); 
            _subs_depth = priv.subscribe(_depth_topic, 1, &InstanceRecognition3D::_process_depth, this); 
            _subs_rgb = priv.subscribe(_rgb_topic, 1, &InstanceRecognition3D::_process_rgb, this); 
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

void InstanceRecognition3D::_process_point(const pcl::PCLPointCloud2::ConstPtr &point_cloud_in){

    if(!_is_on) return;
    cloud_in=point_cloud_in;
    ready_point = true;
}

 void InstanceRecognition3D::_process_rgb(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    rgb_in=img;
    // image_frameid = img->header.frame_id;
    ready_rgb = true;
}

 void InstanceRecognition3D::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    image_frameid = img->header.frame_id;
    ready_depth = true;
}

bool InstanceRecognition3D::loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  std::ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}


bool InstanceRecognition3D::load_database() {
	std::stringstream path, ph5_file, plist_file_name, pkdtree;
	path<<db_path<<"/manipulable_objects/VFH";


	ph5_file<<path.str()<<"/"<<training_data_h5_file_name;
	plist_file_name<<path.str()<<"/"<<training_data_list_file_name;
	pkdtree<<path.str()<<"/"<<kdtree_idx_file_name;

	if (!boost::filesystem::exists (ph5_file.str()) ){
		ROS_ERROR_STREAM("File "<<ph5_file.str()<<" not found.");
		return false;
	}
	if (!boost::filesystem::exists ( plist_file_name.str()) ){
		ROS_ERROR_STREAM("File "<<plist_file_name.str()<<" not found.");
		return false;
	}
	if (!boost::filesystem::exists (pkdtree.str())){
		ROS_ERROR_STREAM("File "<<pkdtree.str()<<" not found.");
		return false;
	}

    flann::Matrix<float> data;
	loadFileList (models, plist_file_name.str());
    flann::load_from_file (data, ph5_file.str(), "training_data");

    index = new flann::Index<flann::ChiSquareDistance<float> > (data, flann::SavedIndexParams (pkdtree.str()));
    index->buildIndex ();
    //nearestKSearch (index, histogram, k, k_indices, k_distances);
}


void InstanceRecognition3D::filter_voxel(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out) {
	pcl::VoxelGrid<Point> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
	sor.filter(*cloud_out);

	ROS_INFO_STREAM("[filter_voxel] Initial points " << cloud_in->points.size() << " - Final points  " << cloud_out->points.size());
}

void InstanceRecognition3D::filter_passthrough(PointCloud cloud_in, PointCloud::Ptr cloud_out) {
    PointCloud::Ptr cloud_ptr  = cloud_in.makeShared();
    pcl::PassThrough<Point> pass;

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

void InstanceRecognition3D::extract_planar(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out) {

  pcl::SACSegmentation<Point> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<Point>::Ptr cloudObj(new pcl::PointCloud<Point>());

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
    pcl::ExtractIndices<Point> extract;
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

std::vector<pcl::PointCloud<Point>::Ptr> InstanceRecognition3D::get_clusters(PointCloud::Ptr  cloud_in) {
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(15);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_in);
  ec.extract(cluster_indices);

  ROS_INFO_STREAM("[get_clusters] " << cluster_indices.size() << " detected  ");

  // Put clusters in a pointcloud vector
  std::vector<pcl::PointCloud<Point>::Ptr> objCluster(cluster_indices.size());
  for (std::size_t i = 0; i < cluster_indices.size(); ++i)
  {
    objCluster[i].reset(new pcl::PointCloud<Point>());
    objCluster[i]->points.reserve(cluster_indices[i].indices.size());
    for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin();
        pit != cluster_indices[i].indices.end(); ++pit)
    {
      objCluster[i]->points.push_back(cloud_in->points[*pit]);
    }
    objCluster[i]->width = objCluster[i]->points.size();
    objCluster[i]->height = 1;
    objCluster[i]->is_dense = true;
    objCluster[i]->header =  cloud_in->header;
    ROS_INFO_STREAM("Cluster " << i+1 << " with " << objCluster[i]->points.size());
  }

  return objCluster;
}

PNormal::Ptr InstanceRecognition3D::get_normals(PointCloud::Ptr cloud_in) {

  PNormal::Ptr cloud_normals (new PNormal);
  pcl::NormalEstimation<Point, pcl::Normal> ne;
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());

  ne.setInputCloud (cloud_in);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);

  return cloud_normals;
}

PVFH::Ptr InstanceRecognition3D::descriptor_vfh(PointCloud::Ptr cloud_in, PNormal::Ptr cloud_normals) {

  PVFH::Ptr vfhs (new PVFH ());
  pcl::VFHEstimation<Point, pcl::Normal, VFH> vfh;
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());

  vfh.setInputCloud (cloud_in);
  vfh.setInputNormals (cloud_normals);
  vfh.setSearchMethod (tree);
  vfh.compute (*vfhs);

  return vfhs;
}

PFPFH::Ptr InstanceRecognition3D::descriptor_fpfh(PointCloud::Ptr cloud_in, PNormal::Ptr cloud_normals) {

  PFPFH::Ptr fpfhs (new PFPFH ());
  pcl::FPFHEstimation<Point, pcl::Normal, pcl::FPFHSignature33> fpfh;
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);

  fpfh.setInputCloud (cloud_in);
  fpfh.setInputNormals (cloud_normals);
  fpfh.setSearchMethod (tree);
  fpfh.setRadiusSearch (0.05);
  fpfh.compute (*fpfhs);

  return fpfhs;
}

 bool InstanceRecognition3D::matching(PVFH dscr_in) {

	vfh_model model;

	model.second.resize (308);
	for (size_t i = 0; i < 308; ++i)
		model.second[i] = dscr_in.points[0].histogram[i];
	
	model.first = "test";


	flann::Matrix<int> k_indices;
	flann::Matrix<float> k_distances;
	int k=6;


	// Query point
	flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
	memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

	k_indices = flann::Matrix<int>(new int[k], 1, k);
	k_distances = flann::Matrix<float>(new float[k], 1, k);
	index->knnSearch (p, k_indices, k_distances, k, flann::SearchParams (512));
	delete[] p.ptr ();



	// Output the results on screen
	std::cout<<"The closest "<<k<<" neighbors are:"<<std::endl;
	for (int i = 0; i < k; ++i)
		std::cout<<i+1<<"  "<<models.at (k_indices[0][i]).first.c_str ()<<"  d:"<< k_distances[0][i]<<std::endl;


 	return true;
}




// - - - - -  RUN   - - - - - - - - - -
void InstanceRecognition3D::run() {
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

    // iterators
    PointCloud::iterator c_it;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step1: Transform cloud to '_base_frame'
  
    // try {
    //     cloud_helper->header.frame_id = _depth_frame;
    //     bool success_transformation = pcl_ros::transformPointCloud(_base_frame, *cloud_helper, transformed_cloud, *_tf_listener);
    //     if (!success_transformation) return;

    // } catch (tf::TransformException & ex) {
    //     ROS_ERROR_STREAM("[" << _name << "] Failed to transform rgbd pointcloud2 in '"
    //             << cloud_helper->header.frame_id << "' frame, to '" << _base_frame << "' : " << ex.what());
    //     return;
    // }


    // Step2: Crop table, robot and walls
    filter_passthrough(*cloud_helper, filter1_cloud);
    filter_voxel(filter1_cloud, filter2_cloud);

    // Step3: Planar extraction and get the objects in different clusters
	extract_planar(filter2_cloud, objects_cloud);
	std::vector<pcl::PointCloud<Point>::Ptr> obj_clusters = get_clusters(objects_cloud);


	int form_id;
	for (std::size_t i = 0; i < obj_clusters.size(); ++i)
	{
		if (obj_clusters[i]->size() < 15)
		{
			ROS_WARN("Cluster with less than 15 points!");
			continue;
		}

	    PNormal::Ptr cluster_normals = get_normals(obj_clusters[i]);
	    PVFH::Ptr cluster_vfh;
	    if (_descriptor == descriptors[0]) 
	      cluster_vfh = descriptor_vfh(obj_clusters[i], cluster_normals); 
	    else if (_descriptor == descriptors[1])
	      PFPFH::Ptr cluster_fpfh = descriptor_fpfh(obj_clusters[i], cluster_normals); 
	  	matching(*cluster_vfh);
	}



}




} /* namespace rhome_perception */

int main(int argc, char** argv) {

    ros::init(argc, argv, "instance_recognition_3d");
    ros::NodeHandle priv("~");

    boost::scoped_ptr<rhome_perception::InstanceRecognition3D> node(
            new rhome_perception::InstanceRecognition3D(ros::this_node::getName())
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
