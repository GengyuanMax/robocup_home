#include <object_prediction.h>

namespace rhome_perception {

ObjectPrediction::ObjectPrediction(string name):_name(name){

    ros::NodeHandle priv("~"), nh;
    // - - - - - - - - l i s t e n e r s - - - - - - - - - - - -
    // - - - - - - - p a r a m e t e r s - - - - - - - - - - -

    priv.param<std::string>("base_frame",_base_frame,"base_link");
    priv.param<std::string>("rgbd_frame",_rgbd_frame,"xtion2_link");
    priv.param<std::string>("rgb_frame",_rgb_frame,"xtion2_rgb_optical_frame");
    priv.param<std::string>("depth_frame",_depth_frame,"xtion2_depth_optical_frame");
    priv.param<std::string>("rgb_topic",_rgb_topic,"/xtion2/rgb/image_raw");
    priv.param<std::string>("depth_topic",_depth_topic,"/xtion2/depth/image_raw");
    priv.param<std::string>("algorithm",_algorithm,"Kalman");

    algorithms = {"Kalman"};

    // if (_algorithm == recognition_alg[0])       lbp = new lbplibrary::OLBP;
    // else {
    //     lbp = new lbplibrary::OLBP;
    // }
    // ROS_INFO_STREAM(" Using "+_algorithm+" algorithm");
    // if (_algorithm == tracking_alg[7])
    //     tracker = cv::TrackerCSRT::create();

    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    // int type = cv::CV_32F;

	kf = cv::KalmanFilter(stateSize, measSize, contrSize, CV_32F);
    state = cv::Mat::zeros(stateSize, 1, CV_32F); /* (phi, delta_phi) */  // [x,y,v_x,v_y,w,h]
    processNoise = cv::Mat::zeros(2, 1, CV_32F);
    measurement = cv::Mat::zeros(1, 1, CV_32F);
    object_meas = cv::Mat::zeros(measSize, 1, CV_32F); // [z_x,z_y,z_w,z_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));


    newposition = false;
    initialized = false;

    ready_rgb = false;
    ready_depth = false;

    // - - - - - - - - p u b l i s h e r s  - - - - - - - - - - - -



    // - - - - - - - - s e r v i c e s  - - - - - - - - - - - - -
    _active_server  = priv.advertiseService("active", &ObjectPrediction::_active_service, this);
}

ObjectPrediction::~ObjectPrediction() {
}

 bool ObjectPrediction::_active_service(rhome_srvs::Onoff::Request  &req, rhome_srvs::Onoff::Response &res) {
 	ticks = 0;

    if(req.select == true) {
        if (!_is_on) {
            _subs_depth = priv.subscribe(_depth_topic, 1, &ObjectPrediction::_process_depth, this); 
            _subs_rgb = priv.subscribe(_rgb_topic, 1, &ObjectPrediction::_process_rgb, this); 
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


void ObjectPrediction::_process_rgb(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    rgb_in=img;
    // image_frameid = img->header.frame_id;
    ImageIn  = cv_bridge::toCvCopy((rgb_in), sensor_msgs::image_encodings::BGR8)->image;
    if (rgb_in->height * rgb_in->width > 0)
        ready_rgb = true;
}

void ObjectPrediction::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    image_frameid = img->header.frame_id;
    DepthIn = cv_bridge::toCvShare(depth_in)->image;
    if (depth_in->height * depth_in->width > 0)
        ready_depth = true;

}

cv::Point ObjectPrediction::calcPoint(cv::Point2f center, double R, double angle)
{
    return center + cv::Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}

bool ObjectPrediction::object_detection(cv::Mat image_in) //TODO change to subscribe to object detection
{

	        // >>>>> Noise smoothing
        cv::Mat blur;
        cv::GaussianBlur(image_in, blur, cv::Size(5, 5), 3.0, 3.0);
        // <<<<< Noise smoothing

        // >>>>> HSV conversion
        cv::Mat frmHsv;
        cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
        // <<<<< HSV conversion

        // >>>>> Color Thresholding
        // Note: change parameters for different colors
        cv::Mat rangeRes = cv::Mat::zeros(image_in.size(), CV_8UC1);
        cv::inRange(frmHsv, cv::Scalar(MIN_H_YELLOW , 60, 100),
                    cv::Scalar(MAX_H_YELLOW, 255, 255), rangeRes);
        // <<<<< Color Thresholding

        // >>>>> Improving the result
        cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
        // <<<<< Improving the result

        // Thresholding viewing
        cv::imshow("Threshold", rangeRes);

        // >>>>> Contours detection
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,
                         CV_CHAIN_APPROX_NONE);
        // <<<<< Contours detection

        // >>>>> Filtering
        std::vector<std::vector<cv::Point> > balls;
        std::vector<cv::Rect> ballsBox;
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::Rect bBox;
            bBox = cv::boundingRect(contours[i]);

            float ratio = (float) bBox.width / (float) bBox.height;
            if (ratio > 1.0f)
                ratio = 1.0f / ratio;

            // Searching for a bBox almost square
            if (ratio > 0.75 && bBox.area() >= 400)
            {
                balls.push_back(contours[i]);
                ballsBox.push_back(bBox);
            }
        }
        // <<<<< Filtering

        std::cout << "Balls found:" << ballsBox.size() << std::endl;

        if (balls.size() > 0)
        {
            object_point.x = ballsBox[0].x + ballsBox[0].width / 2;
            object_point.y = ballsBox[0].y + ballsBox[0].height / 2;
            object_meas.at<float>(0) = object_point.x;
            object_meas.at<float>(1) = object_point.y;
            object_meas.at<float>(2) = (float)ballsBox[0].width;
            object_meas.at<float>(3) = (float)ballsBox[0].height;

        	newposition =true;
        	return true;
        }
        return false;
        // // >>>>> Detection result
        // for (size_t i = 0; i < balls.size(); i++)
        // {
        //     cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
        //     cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

        //     cv::Point center;
        //     center.x = ballsBox[i].x + ballsBox[i].width / 2;
        //     center.y = ballsBox[i].y + ballsBox[i].height / 2;
        //     cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

        //     stringstream sstr;
        //     sstr << "(" << center.x << "," << center.y << ")";
        //     cv::putText(res, sstr.str(),
        //                 cv::Point(center.x + 3, center.y - 3),
        //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
        // }
        // <<<<< Detection result
}

// - - - - -  RUN   - - - - - - - - - -
void ObjectPrediction::run() {
    ready_rgb = false;
    ready_depth = false;

    if (!object_detection(ImageIn) && !initialized)
    	return;

    double precTick = ticks, dT;
    ticks = (double) cv::getTickCount();
    dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

    if (!initialized) // First detection!
    {
        // >>>> Initialization
        kf.errorCovPre.at<float>(0) = 1; // px
        kf.errorCovPre.at<float>(7) = 1; // px
        kf.errorCovPre.at<float>(14) = 1;
        kf.errorCovPre.at<float>(21) = 1;
        kf.errorCovPre.at<float>(28) = 1; // px
        kf.errorCovPre.at<float>(35) = 1; // px

        state.at<float>(0) = object_meas.at<float>(0);
        state.at<float>(1) = object_meas.at<float>(1);
        state.at<float>(2) = 0;
        state.at<float>(3) = 0;
        state.at<float>(4) = object_meas.at<float>(2);
        state.at<float>(5) = object_meas.at<float>(3);

        kf.statePost = state;
        initialized = true;
    }
    kf.transitionMatrix.at<float>(2) = dT;
    kf.transitionMatrix.at<float>(9) = dT;

    state = kf.predict();
    // cout << "State post:" << endl << state << endl;

    cv::Rect predRect;
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0) - predRect.width / 2;
    predRect.y = state.at<float>(1) - predRect.height / 2;

	cv::rectangle(ImageIn, predRect, CV_RGB(255,0,0), 2);
	cv::imshow( "Kalman", ImageIn );
    cv::waitKey(10);

    //Kalman Update
    kf.correct(object_meas); // Kalman Correction

}




} /* namespace rhome_perception */

int main(int argc, char** argv) {

    ros::init(argc, argv, "object_prediction");
    ros::NodeHandle priv("~");

    boost::scoped_ptr<rhome_perception::ObjectPrediction> node(
            new rhome_perception::ObjectPrediction(ros::this_node::getName())
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
