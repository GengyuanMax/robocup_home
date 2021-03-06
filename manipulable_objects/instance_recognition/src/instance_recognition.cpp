#include <instance_recognition.h>

namespace rhome_perception {

InstanceRecognition::InstanceRecognition(string name):_name(name){

    ros::NodeHandle priv("~"), nh;
    // - - - - - - - - l i s t e n e r s - - - - - - - - - - - -
    // - - - - - - - p a r a m e t e r s - - - - - - - - - - -

    priv.param<std::string>("base_frame",_base_frame,"base_link");
    priv.param<std::string>("rgbd_frame",_rgbd_frame,"xtion2_link");
    priv.param<std::string>("rgb_frame",_rgb_frame,"xtion2_rgb_optical_frame");
    priv.param<std::string>("depth_frame",_depth_frame,"xtion2_depth_optical_frame");
    priv.param<std::string>("rgb_topic",_rgb_topic,"/xtion2/rgb/image_raw");
    priv.param<std::string>("depth_topic",_depth_topic,"/xtion2/depth/image_raw");
    priv.param<std::string>("depth_info",_depth_info_topic,"/camera/depth/camera_info");
    priv.param<std::string>("algorithm",_algorithm,"LBP");
    priv.param<std::string>("compare_hist",_compare_hist,"correlation");

    recognition_alg = {"LBP", "ELBP", "SIFT", "SURF", "ORB"};
    compare_hist_methods = {"correlation", "chisquare", "intersection", "bhattacharyya"};
//ORB, AKAZE https://docs.opencv.org/3.0-beta/doc/tutorials/features2d/akaze_tracking/akaze_tracking.html#akazetracking


    if (_algorithm == recognition_alg[0])       lbp = new lbplibrary::OLBP;
    else if (_algorithm == recognition_alg[1])  lbp = new lbplibrary::ELBP;
    else if (_algorithm == recognition_alg[2]){
        detector = cv::xfeatures2d::SIFT::create(); 
        extractor = cv::xfeatures2d::SIFT::create();
    }else if (_algorithm == recognition_alg[3]){
        detector = cv::xfeatures2d::SURF::create(400); //cv::SurfFeatureDetector (400);
        extractor = cv::xfeatures2d::SURF::create();
    }else if (_algorithm == recognition_alg[4]){
        detector = cv::ORB::create(); 
        extractor = cv::ORB::create();
    }else {
        lbp = new lbplibrary::OLBP;
    }
    ROS_INFO_STREAM(" Using "+_algorithm+" algorithm");

    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    if (_algorithm == recognition_alg[4])
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);


	db_path = ros::package::getPath("perception_databases");
	if (db_path.size() == 0)
		ROS_ERROR_STREAM("Package perception_databases don't found.");
	else{
		load_database();
	}

    ready_rgb = false;
    ready_depth = false;

    // - - - - - - - - p u b l i s h e r s  - - - - - - - - - - - -

    // - - - - - - - - s u b s c r i b e r s  - - - - - - - - - - - -
    _subs_dinfo = priv.subscribe(_depth_info_topic, 1, &InstanceRecognition::_process_depthinfo, this); 


    // - - - - - - - - s e r v i c e s  - - - - - - - - - - - - -
    _active_server  = priv.advertiseService("active", &InstanceRecognition::_active_service, this);
    _getobjects_server  = priv.advertiseService("get_objects", &InstanceRecognition::getobjects_service, this);
    _queryobjects_server  = priv.advertiseService("query_objects", &InstanceRecognition::queryobjects_service, this);
}

InstanceRecognition::~InstanceRecognition() {
}

 bool InstanceRecognition::_active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) {

    if (_algorithm == recognition_alg[0] || _algorithm == recognition_alg[1]){
        ROS_ERROR_STREAM("The descriptor "<< _algorithm<<"doesn't work with the sensor. Please try with one of these descriptors: "<<recognition_alg[2]<<", "<<recognition_alg[3]<<" or "<<recognition_alg[4]);
        return false;
    } 

    if(req.select == true) {
        if (!_is_on) {
            _subs_depth = priv.subscribe(_depth_topic, 1, &InstanceRecognition::_process_depth, this); 
            _subs_rgb = priv.subscribe(_rgb_topic, 1, &InstanceRecognition::_process_rgb, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+_depth_topic+". . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_depth.shutdown();
              _is_on = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}

bool InstanceRecognition::getobjects_service(rhome_srvs::Getinformation::Request  &req, rhome_srvs::Getinformation::Response &res) {

    if (!_is_on){
        ROS_INFO_STREAM("Please, turne on the node before call this service.");
        return true;
    }

//    res.pose = detected_pose;  take from depth_img
    for (int i = 0; i < detected_imgs.size(); ++i)
    {
        cv_bridge::CvImage out_msg;
        out_msg.header   = rgb_in->header; 
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; 
        out_msg.image    = detected_imgs[i];
        sensor_msgs::Image msg = *out_msg.toImageMsg();
        res.imgs.push_back (msg);
        int c_x = (detected_roi[i].p1.x + detected_roi[i].p2.x + detected_roi[i].p3.x + detected_roi[i].p4.x)/4;
        int c_y = (detected_roi[i].p1.y + detected_roi[i].p2.y + detected_roi[i].p3.y + detected_roi[i].p4.y)/4;
        res.pose.push_back (point2d_to_pose(cv::Point(c_x, c_y)));
    }

    res.roi = detected_roi;
    res.inf_label = detected_label;

    return true;
}


bool InstanceRecognition::queryobjects_service(rhome_srvs::Queryobject::Request  &req, rhome_srvs::Queryobject::Response &res) {

    for (int i = 0; i < req.imgs.size(); ++i)
    {    
        std::vector<cv::KeyPoint> kp_test;
        cv::Mat img = cv_bridge::toCvCopy((req.imgs[i]), sensor_msgs::image_encodings::BGR8)->image;
        cv::Mat test_dscr = extract_dscr(img, kp_test);

        std::vector< std::vector<cv::DMatch> > matches;
        std::vector<int>  match_ids = matching_db(test_dscr, matches);
        if ( match_ids.size() > 0)
            res.inf_label.push_back(db_label[match_ids[0]]);
        else
            res.inf_label.push_back("undetected");
       
    }
    return true;
}



void InstanceRecognition::_process_rgb(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    rgb_in=img;
    // image_frameid = img->header.frame_id;
    ImageIn  = cv_bridge::toCvCopy((rgb_in), sensor_msgs::image_encodings::BGR8)->image;
    if (rgb_in->height * rgb_in->width > 0)
        ready_rgb = true;
}

void InstanceRecognition::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    image_frameid = img->header.frame_id;
    DepthIn = cv_bridge::toCvShare(depth_in)->image;
    if (depth_in->height * depth_in->width > 0)
        ready_depth = true;

}

void InstanceRecognition::_process_depthinfo(const sensor_msgs::CameraInfo& info){
//Exmaple : K: [618.5377197265625, 0.0, 313.529052734375, 0.0, 618.5377807617188, 237.82754516601562, 0.0, 0.0, 1.0]
    colorcamera_info.push_back(info.K[0]);
    colorcamera_info.push_back(info.K[4]);
    colorcamera_info.push_back(info.K[2]);
    colorcamera_info.push_back(info.K[5]);

    _subs_dinfo.shutdown();
}

bool InstanceRecognition::load_database() {
	std::stringstream path_db;
	path_db<<db_path<<"/manipulable_objects/Objects";

    path directory_train(path_db.str());

    directory_iterator end_it;
    file_status f = status(directory_train);

    if (!(f.type() != status_unknown && f.type() != file_not_found))
        ROS_ERROR_STREAM("Training directory not found");

   // bool only_one = false;
    for( directory_iterator it( directory_train ); it != end_it; it++ ) {
        if ( is_directory( it->status() ) )
        {
    

            std::stringstream path_obj;
            path_obj<<path_db.str()<<"/"<<it->path().filename().c_str();
            path obj(path_obj.str());

            std::string name_object = it->path().filename().c_str();

            int n_imgs=0;
            for( directory_iterator it2( obj ); it2 != end_it; it2++ ) {
                if( is_regular_file( it2->status()) && (it2->path().extension() == ".jpg" || it2->path().extension() == ".png"))
                {
                    std::stringstream path_img;
                    std::string img=it2->path().filename().c_str();
                    path_img<<path_obj.str()<<"/"<<img;
                    ROS_INFO_STREAM("file "<<path_img.str());

                    cv::Mat mat_img = cv::imread(path_img.str(), CV_LOAD_IMAGE_COLOR); 
                    std::vector<cv::KeyPoint> kp;
                    cv::Mat dscr = extract_dscr(mat_img, kp);
                    db_kp.push_back(kp);
                    db_dscr.push_back(dscr);
                    db_label.push_back(name_object);
                    db_heigth.push_back(mat_img.rows);
                    db_width.push_back(mat_img.cols);

                    train_succes = true;
                    n_imgs ++;
                }
            }
            ROS_INFO_STREAM("Object "+name_object+". . . "<< n_imgs <<" Images Trained");
            
        }
    
    }


    if (train_succes)
    {
        ROS_INFO_STREAM("Training Success. "<<db_label.size()<< " Images in Total");
    }
    else
         ROS_INFO_STREAM("Training Failed. Please check the directory: "<<path_db.str());

    _is_on = false;

    return true;

}


cv::Mat  InstanceRecognition::get_histogram(cv::Mat1b const& image)
  {
    // Set histogram bins count
    int bins = 256;
    int histSize[] = { bins };
    // Set ranges for histogram bins
    float lranges[] = { 0, 256 };
    const float* ranges[] = { lranges };
    // create matrix for histogram
    cv::Mat hist;
    int channels[] = { 0 };

    // create matrix for histogram visualization
    int const hist_height = 256;
    cv::Mat3b hist_image = cv::Mat3b::zeros(hist_height, bins);

    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1, histSize, ranges, true, false);

    double max_val = 0;
    minMaxLoc(hist, 0, &max_val);
    // // Print histogram
    // std::cout<< "LBP histogram : ";
    // for (int b = 0; b < bins; b++) 
    //     std::cout<< hist.at<float>(b)<<" ";
    // std::cout<< std::endl;

    // // visualize each bin
    // for (int b = 0; b < bins; b++) {
    //   float const binVal = hist.at<float>(b);
    //   int   const height = cvRound(binVal*hist_height / max_val);
    //   cv::line
    //     (hist_image
    //     , cv::Point(b, hist_height - height), cv::Point(b, hist_height)
    //     , cv::Scalar::all(255)
    //     );
    // }
    // // cv::imshow(name, hist_image);
    return hist;
  }


cv::Mat InstanceRecognition::extract_dscr(cv::Mat image_input, std::vector<cv::KeyPoint> &kp) {

    cv::Mat dscr;

    if (_algorithm == recognition_alg[0]) {
        cv::Mat img_lbp;
        lbp->run(image_input, img_lbp);
        cv::normalize(img_lbp, img_lbp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        dscr = get_histogram(img_lbp);
    }
    else {
        detector->detect(image_input, kp);
        extractor->compute(image_input, kp, dscr);
    }

    return dscr;

}

std::vector<int> InstanceRecognition::matching_db(cv::Mat dscr_input, std::vector< std::vector<cv::DMatch> > &matches) {

    const float ratio_thresh = 0.7f;
    std::vector<int> n_matches, id_matches;

    if (_algorithm == recognition_alg[0] || _algorithm == recognition_alg[1]){
        int best_match = 0, best_id;
        for (int i = 0; i < db_dscr.size(); ++i)
        {
            double value_match;
            if(_compare_hist == compare_hist_methods[0]) value_match = cv::compareHist(db_dscr[i], dscr_input, CV_COMP_CORREL);  
            if(_compare_hist == compare_hist_methods[1]) value_match = cv::compareHist(db_dscr[i], dscr_input, CV_COMP_CHISQR);   
            if(_compare_hist == compare_hist_methods[2]) value_match = cv::compareHist(db_dscr[i], dscr_input, CV_COMP_INTERSECT);   
            if(_compare_hist == compare_hist_methods[3]) value_match = cv::compareHist(db_dscr[i], dscr_input, CV_COMP_BHATTACHARYYA);   

            if (value_match > best_match)  
            {
                best_match = value_match;
                best_id = i;
            }
        }
        if (best_match > 0.5)//confirm
            id_matches.push_back(best_id);

        return id_matches;
    } 
    

    int best_match = 0, best_id;
    for (int i = 0; i < db_dscr.size(); ++i)
    {
        std::vector< std::vector<cv::DMatch> > knn_matches;
        std::vector<cv::DMatch> good_matches;
        matcher->knnMatch( db_dscr[i], dscr_input, knn_matches, 2 ); //2?

        for (size_t j = 0; j < knn_matches.size(); j++)
        {
            if (knn_matches[j][0].distance < ratio_thresh * knn_matches[j][1].distance)
                good_matches.push_back(knn_matches[j][0]);
        }
        matches.push_back(good_matches);

        if (good_matches.size() > best_match)  
        {
             best_match = good_matches.size();
             best_id = i;
        }
        n_matches.push_back(good_matches.size());
    }

    //Printing matching information
    //std::cout<<"best match: "<<db_label[best_id]<<std::endl;
    for (int i = 0; i < n_matches.size(); ++i)
        if (n_matches[i] >= 9){
         //   std::cout<<"match: "<<db_label[i] <<" - "<< n_matches[i]<<std::endl;
            id_matches.push_back(i);
        }


    return id_matches;

}

void InstanceRecognition::remove_duplicates(std::vector<box_obj> &objs, std::vector<cv::Rect> &objs_rects){
    for (int i = 1; i < objs.size(); ++i)
    {
        cv::Rect r1 = cv::Rect(std::min(objs[i].p1.x, objs[i].p3.x), std::min(objs[i].p1.y, objs[i].p3.y), std::max(objs[i].p2.x, objs[i].p4.x) - std::min(objs[i].p1.x, objs[i].p3.x), std::max(objs[i].p2.y, objs[i].p4.y) - std::min(objs[i].p1.y, objs[i].p3.y));
        for (int j = 0; j < i; ++j)
        {
            cv::Rect r2 = cv::Rect(std::min(objs[j].p1.x, objs[j].p3.x), std::min(objs[j].p1.y, objs[j].p3.y), std::max(objs[j].p2.x, objs[j].p4.x) - std::min(objs[j].p1.x, objs[j].p3.x), std::max(objs[j].p2.y, objs[j].p4.y) - std::min(objs[j].p1.y, objs[j].p3.y));

            cv::Rect intersection = r1 & r2;
            if (intersection.width * intersection.height > r1.width * r1.height * 0.4){
                if (r1.width * r1.height > r2.width * r2.height ) objs.erase(objs.begin() + i);
                else objs.erase(objs.begin() + j);

                i--;
                continue;
            }
        }
    }
    
    for (int i = 0; i < objs.size(); ++i)
    {
        cv::Rect r1 = cv::Rect(std::min(objs[i].p1.x, objs[i].p3.x), std::min(objs[i].p1.y, objs[i].p3.y), std::max(objs[i].p2.x, objs[i].p4.x) - std::min(objs[i].p1.x, objs[i].p3.x), std::max(objs[i].p2.y, objs[i].p4.y) - std::min(objs[i].p1.y, objs[i].p3.y));
        objs_rects.push_back(r1);
    }

}

void InstanceRecognition::point_to_position(cv::Point point2d, geometry_msgs::Point &point3d){
    point3d.x = point2d.x;
    point3d.y = point2d.y;
}
    

geometry_msgs::PoseStamped InstanceRecognition::point2d_to_pose(cv::Point point_in){

      if(! DepthIn.data || DepthIn.cols==0 || DepthIn.rows==0)
        ROS_ERROR_STREAM("geometry_msgs::PoseStamped InstanceRecognition::point2d_to_pose() : depth image null");


    geometry_msgs::PoseStamped pose_out;

    float fx = colorcamera_info[0], fy = colorcamera_info[1];
    float cx = colorcamera_info[2], cy = colorcamera_info[3];

    uint16_t z_raw = DepthIn.at<uint16_t>(point_in.y, point_in.x);
    pose_out.pose.position.z = z_raw * 0.001;
    pose_out.pose.position.x = (point_in.x - cx) * pose_out.pose.position.z / fx;
    pose_out.pose.position.y = (point_in.y - cy) * pose_out.pose.position.z / fy;
    pose_out.pose.orientation.w = 1;

    return pose_out;

}


// - - - - -  RUN   - - - - - - - - - -
void InstanceRecognition::run() {
    ready_rgb = false;
    ready_depth = false;

    std::vector<cv::KeyPoint> kp_test;
    cv::Mat test_dscr = extract_dscr(ImageIn, kp_test);

    std::vector< std::vector<cv::DMatch> > matches;
    std::vector<int>  match_ids = matching_db(test_dscr, matches);

    std::vector<box_obj> objects;

    for (int i = 0; i < match_ids.size(); ++i)
    {

        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;
        std::vector<cv::Point2f> obj_corners(4), scene_corners(4);
        box_obj box;

        for( size_t j = 0; j < matches[match_ids[i]].size(); j++ )
        {
            obj.push_back( db_kp[match_ids[i]][ matches[match_ids[i]][j].queryIdx ].pt );
            scene.push_back( kp_test[matches[match_ids[i]][j].trainIdx].pt );
        }

        cv::Mat H =  cv::findHomography( obj, scene, cv::RANSAC );
        
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( db_width[match_ids[i]], 0 );
        obj_corners[2] = cvPoint( db_width[match_ids[i]], db_heigth[match_ids[i]] ); obj_corners[3] = cvPoint( 0, db_heigth[match_ids[i]] );

        cv::perspectiveTransform( obj_corners, scene_corners, H);
        box.p1 = scene_corners[0]; box.p2 = scene_corners[1]; 
        box.p3 = scene_corners[2]; box.p4 = scene_corners[3]; 
        objects.push_back(box);
    }
    
    std::vector<cv::Rect> objs_rects;
    remove_duplicates(objects, objs_rects);

    //Saving data
    detected_imgs.clear();
    detected_label.clear();
    detected_roi.clear();
    for (int i = 0; i < objects.size(); ++i)
    {
        cv::line( ImageIn, objects[i].p1, objects[i].p2, cv::Scalar(255, 0, 0), 4 );
        cv::line( ImageIn, objects[i].p2, objects[i].p3, cv::Scalar( 255, 0, 0), 4 );
        cv::line( ImageIn, objects[i].p3, objects[i].p4, cv::Scalar( 255, 0, 0), 4 );
        cv::line( ImageIn, objects[i].p4, objects[i].p1, cv::Scalar( 255, 0, 0), 4 );

        detected_imgs.push_back( ImageIn(objs_rects[i]) );
        detected_label.push_back (db_label[match_ids[i]]);

        rhome_msgs::Roi obj_roi;
        point_to_position(objects[i].p1, obj_roi.p1);
        point_to_position(objects[i].p2, obj_roi.p2);
        point_to_position(objects[i].p3, obj_roi.p3);
        point_to_position(objects[i].p4, obj_roi.p4);
        detected_roi.push_back (obj_roi);
    }
    depth_img = DepthIn;

    cv::imshow("InstanceRecognition", ImageIn); 
    cv::waitKey(5);
}




} /* namespace rhome_perception */

int main(int argc, char** argv) {

    ros::init(argc, argv, "instance_recognition");
    ros::NodeHandle priv("~");

    boost::scoped_ptr<rhome_perception::InstanceRecognition> node(
            new rhome_perception::InstanceRecognition(ros::this_node::getName())
    );


    int _fps = 25;
    if(!priv.getParam("fps",_fps)) priv.setParam("fps",_fps);

    ros::Rate r(_fps);

    while(ros::ok()){
        if (node->_is_on && node->ready_depth && node->ready_rgb)
          node->run();
          
        // r.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Quitting ... \n");

    return 0;
}
