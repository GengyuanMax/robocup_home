#include <lrhome_3d.h>

namespace rhome_perception {

LRHOME3D::LRHOME3D(){

    descriptors = {"VFH", "FPFH"};

    ROS_INFO_STREAM(" Using "+_descriptor+" algorithm");

}

LRHOME3D::~LRHOME3D() {
    delete _tf_listener;
}


void LRHOME3D::filter_voxel(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out, double _leaf_size) {
	pcl::VoxelGrid<Point> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
	sor.filter(*cloud_out);

	ROS_INFO_STREAM("[filter_voxel] Initial points " << cloud_in->points.size() << " - Final points  " << cloud_out->points.size());
}

void LRHOME3D::filter_passthrough(PointCloud cloud_in, PointCloud::Ptr cloud_out) {
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

void LRHOME3D::extract_planar(PointCloud::Ptr  cloud_in, PointCloud::Ptr cloud_out) {

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

std::vector<pcl::PointCloud<Point>::Ptr> LRHOME3D::get_clusters(PointCloud::Ptr  cloud_in) {
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

PNormal::Ptr LRHOME3D::get_normals(PointCloud::Ptr cloud_in) {

  PNormal::Ptr cloud_normals (new PNormal);
  pcl::NormalEstimation<Point, pcl::Normal> ne;
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());

  ne.setInputCloud (cloud_in);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);

  return cloud_normals;
}

PVFH::Ptr LRHOME3D::descriptor_vfh(PointCloud::Ptr cloud_in, PNormal::Ptr cloud_normals) {

  PVFH::Ptr vfhs (new PVFH ());
  pcl::VFHEstimation<Point, pcl::Normal, VFH> vfh;
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());

  vfh.setInputCloud (cloud_in);
  vfh.setInputNormals (cloud_normals);
  vfh.setSearchMethod (tree);
  vfh.compute (*vfhs);

  return vfhs;
}

PFPFH::Ptr LRHOME3D::descriptor_fpfh(PointCloud::Ptr cloud_in, PNormal::Ptr cloud_normals) {

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




} /* namespace rhome_perception */
