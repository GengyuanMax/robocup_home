#include <instance_training_3d.h>

namespace rhome_perception {

InstanceTraining3D::InstanceTraining3D(string name):_name(name){

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
    priv.param<bool>("save_pointclouds", _save_pointclouds, true);
    priv.param<float>("crop_width", _crop_width, 2.0);
    priv.param<float>("crop_depth", _crop_depth, 2.0);
    priv.param<float>("crop_min_z", _crop_min_z, -2.0);
    priv.param<float>("crop_max_z", _crop_max_z, 2.0);
    priv.param<float>("leaf_size", _leaf_size, 0.008);
    priv.param<std::string>("descriptor",_descriptor, "VFH");


    descriptors = {"VFH", "FPFH"};

    ROS_INFO_STREAM(" Using "+_descriptor+" algorithm"); //TODO possible extract a list of features

    cv::namedWindow("img", 1);
    pt = cv::Point(-1,-1);
    newCoords = false;
    stop_saving = false;
    name_object = "default";

    db_path = ros::package::getPath("perception_databases");
	kdtree_idx_file_name = "kdtree.idx";
	training_data_h5_file_name = "training_data.h5";
	training_data_list_file_name = "training_data.list";

	if (db_path.size() ==0)
		ROS_ERROR_STREAM("Package perception_databases don't found. Please, create this package to training objects.");

	std::stringstream path;
	path<<db_path<<"/manipulable_objects/VFH";
	if (!boost::filesystem::exists(path.str()))
		boost::filesystem::create_directories(path.str());

    // - - - - - - - - p u b l i s h e r s  - - - - - - - - - - - -
    cv::setMouseCallback("img", InstanceTraining3D::mouse_callback, (void*)this);
    // - - - - - - - - s e r v i c e s  - - - - - - - - - - - - -
    _active_server  = priv.advertiseService("active", &InstanceTraining3D::_active_service, this);
    _train_server  = priv.advertiseService("train_object", &InstanceTraining3D::_train_service, this);
    _save_server  = priv.advertiseService("save_database", &InstanceTraining3D::_save_database, this);

}

InstanceTraining3D::~InstanceTraining3D() {
    delete _tf_listener;
}

 bool InstanceTraining3D::_active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_point = priv.subscribe(_point_topic, 1, &InstanceTraining3D::_process_point, this); 
            _subs_depth = priv.subscribe(_depth_topic, 1, &InstanceTraining3D::_process_depth, this); 
            _subs_rgb = priv.subscribe(_rgb_topic, 1, &InstanceTraining3D::_process_rgb, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+ _imagePoint_topic+". . . OK");
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

 bool InstanceTraining3D::_train_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) {

  name_object = "test";
  n_object = 1;
  return true;
}


bool InstanceTraining3D::_save_database(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res){

	if (models.size () == 0)
	{
		ROS_ERROR_STREAM("There are no objects to save the database. First, select some object.");
		return true;
	}
	std::stringstream path, ph5_file, plist_file_name, pkdtree;
	path<<db_path<<"/manipulable_objects/VFH";


	ph5_file<<path.str()<<"/"<<training_data_h5_file_name;
	plist_file_name<<path.str()<<"/"<<training_data_list_file_name;
	pkdtree<<path.str()<<"/"<<kdtree_idx_file_name;

	// Convert data into FLANN format
	flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

	for (size_t i = 0; i < data.rows; ++i)
	for (size_t j = 0; j < data.cols; ++j)
	  data[i][j] = models[i].second[j];


	// Save data to disk (list of models)
	flann::save_to_file (data, ph5_file.str(), "training_data");
	std::ofstream fs;
	fs.open ( plist_file_name.str().c_str ());
	for (size_t i = 0; i < models.size (); ++i)
	fs << models[i].first << "\n";
	fs.close ();

	// Build the tree index and save it to disk
	ROS_INFO_STREAM("Building the kdtree index for "<<data.rows<<" elements.");
	flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
	//flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
	index.buildIndex ();
	index.save (pkdtree.str());
	ROS_INFO_STREAM("kdtree index saved.");

	return true;
}


void InstanceTraining3D::_process_point(const pcl::PCLPointCloud2::ConstPtr &point_cloud_in){

    if(!_is_on) return;
    cloud_in=point_cloud_in;
    ready_point = true;
}

 void InstanceTraining3D::_process_rgb(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    rgb_in=img;
    ImageIn  = cv_bridge::toCvCopy((rgb_in), sensor_msgs::image_encodings::BGR8)->image;
    if (rgb_in->height * rgb_in->width > 0)
        ready_rgb = true;
}

 void InstanceTraining3D::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    image_frameid = img->header.frame_id;
    ready_depth = true;
}



void InstanceTraining3D::mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        InstanceTraining3D* thiz = static_cast<InstanceTraining3D*>(param);
        thiz->save_ptmouse (x, y);
    }
}


void InstanceTraining3D::save_ptmouse (int  x, int  y){

        pt.x = x;
        pt.y = y;
        newCoords = true;

}

bool InstanceTraining3D::save_cluster (PointCloud::Ptr cloud_in){

	std::stringstream path, name_cluster;
	path<<db_path<<"/manipulable_objects/VFH/"<<name_object;
	name_cluster<<path.str()<<"/"<<name_object<<"_"<<n_object<<".pcd";;

	if (!boost::filesystem::exists(path.str()))
		boost::filesystem::create_directories(path.str());


	pcl::io::savePCDFile (name_cluster.str(), *cloud_in);

    return true;
}

bool InstanceTraining3D::point2d_in_cloud(cv::Point pt_in, PointCloud::Ptr cloud_in){

    //TODO take values of camera_info
    //TODO transform point of rgb to depth frame
    float cx=308.2192687988281,  cy = 245.59515380859375; //rgb intrinsic
    float fx = 473.1015319824219, fy = 473.1015625;
    int x_min = 640 , x_max = 0, y_min =480, y_max = 0;
//color K: [618.5377197265625, 0.0, 313.529052734375, 0.0, 618.5377807617188, 237.82754516601562, 0.0, 0.0, 1.0]
//depthK: [473.1015319824219, 0.0, 308.2192687988281, 0.0, 473.1015625, 245.59515380859375, 0.0, 0.0, 1.0]

    PointCloud::iterator c_it;
    for ( c_it = cloud_in->begin(); c_it < cloud_in->end(); c_it++) {
        if (std::isfinite(c_it->x) && std::isfinite(c_it->y) && std::isfinite(c_it->z) )
        {
            int x_2d,y_2d;
            float x_3d = c_it->x, y_3d = c_it->y, z_3d = c_it->z;

            x_2d = x_3d*fx/z_3d  + cx;
            y_2d = y_3d*fy/z_3d  + cy;


            if (x_2d < x_min )  x_min = x_2d;
            if (x_2d > x_max )  x_max = x_2d;
            if (y_2d < y_min )  y_min = y_2d;
            if (y_2d > y_max )  y_max = y_2d;

            // if (abs(x_2d-pt_in.x)<10 && abs(y_2d-pt_in.y)<10 )
            //      ROS_INFO("Cluster with the point");
            //     return true;
        }
    }
    int gap = 50;
    if (x_min - gap < pt_in.x && x_max + gap > pt_in.x && y_min - gap < pt_in.y && y_max + gap> pt_in.y)
        return true;
        // ROS_INFO("Cluster with the point(2)");

    // std::cout << "RECT: " << x_min<<" "<< y_min<<" "<< x_max - x_min<<" "<< y_max - y_min << std::endl;
    return false;
}

bool InstanceTraining3D::cluster_model(PVFH cluster_vfh, vfh_model &vfh) {

    vfh.first = name_object;
    vfh.second.resize (308);

    // std::vector <pcl::PCLPointField> fields;
    // pcl::getFieldIndex (cluster_vfh, "vfh", fields);

    //for (size_t i = 0; i < cluster_vfh.points[0].histogram.size(); ++i)  
    for (size_t i = 0; i < 308; ++i)  
        vfh.second[i] = cluster_vfh.points[0].histogram[i];
    

    n_object++;

    return true;
}

bool InstanceTraining3D::save_data(cv::Mat image_in){

 //    // // Clouds
    PointCloud::Ptr cloud_helper      (new PointCloud);
    pcl::fromPCLPointCloud2(*cloud_in, *cloud_helper);
 //    PointCloud transformed_cloud, cloud_process; // transformed cloud
    PointCloud::Ptr filter1_cloud (new PointCloud), filter2_cloud (new PointCloud), objects_cloud (new PointCloud);



 //    // Step2: Crop table, robot and walls
    // lib3d.filter_passthrough(*cloud_helper, filter1_cloud);
    lib3d.filter_voxel(cloud_helper, filter2_cloud, _leaf_size);

 //    // Step3: Planar extraction and get the objects in different clusters
    lib3d.extract_planar(filter2_cloud, objects_cloud);
    std::vector<pcl::PointCloud<Point>::Ptr> obj_clusters = lib3d.get_clusters(objects_cloud);

    bool saved =false;
    // int form_id;
    for (std::size_t i = 0; i < obj_clusters.size(); ++i)
    {
        if (obj_clusters[i]->size() > 15)
        {
            // cv::Rect rect = point2d_in_cloud(pt, obj_clusters[i]);
            // cv::rectangle(image_in, rect, cv::Scalar(0, 255, 0));
            

            if ( point2d_in_cloud(pt, obj_clusters[i]) ) {
                ROS_INFO("Object detected");
                
                if (_save_pointclouds)
                	save_cluster (obj_clusters[i]);

                //https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/vfh_recognition/build_tree.cpp
                PNormal::Ptr cluster_normals = lib3d.get_normals(obj_clusters[i]);
                PVFH::Ptr cluster_vfh;
                if (_descriptor == descriptors[0]) 
                  cluster_vfh = lib3d.descriptor_vfh(obj_clusters[i], cluster_normals); 
            //     // else if (_descriptor == descriptors[1])
            //     //   PFPFH::Ptr cluster_fpfh = descriptor_fpfh(obj_clusters[i], cluster_normals); 
                vfh_model m;
                cluster_model(*cluster_vfh, m);
                models.push_back (m);
                saved = true;

                break;
            }
        }
    }

    if (!saved)
    {
        ROS_ERROR("Object not detected, please try again");
    }
    // cv::imshow("imgrect", image_in);
    // cv::waitKey(0);

    return true;
}

// - - - - -  RUN   - - - - - - - - - -
void InstanceTraining3D::run() {
    if (!ready_point) return;

    if (!stop_saving) {
        cv::imshow("img", ImageIn);
        if (cv::waitKey(10) == 'q') stop_saving = true; // "destroy windows and stop mouse"

        if (pt.x != -1 && pt.y != -1)
        {
          // circle(frame, pt, 3, Scalar(0, 0, 255));

            if (newCoords)
            {
              // std::cout << "Clicked coordinates: " << pt << std::endl;
              save_data(ImageIn);
              newCoords = false;
            }
            pt.x = -1;
            pt.y = -1;
            ready_point = false;
        }
    }


}




} /* namespace rhome_perception */

int main(int argc, char** argv) {

    ros::init(argc, argv, "instance_training_3d");
    ros::NodeHandle priv("~");

    boost::scoped_ptr<rhome_perception::InstanceTraining3D> node(
            new rhome_perception::InstanceTraining3D(ros::this_node::getName())
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
