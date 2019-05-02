#include<turtle_vis/myClass/TurtleClass.h>

namespace turtleSpace {

TurtleClass::TurtleClass()
{
    //#>>>>TODO: INITIALIZE MEMBER VARIABLES
    count_mutex = PTHREAD_MUTEX_INITIALIZER;

    this->turtlePose_g<<0, 0, 0;
    this->turtlePose_desired_g<<0, 0, 0;
}
TurtleClass::~TurtleClass()
{

}

void TurtleClass::getPose(const turtle_vis::DesiredPose::ConstPtr &msg)
{
    pthread_mutex_lock( &this->count_mutex );
    //#>>>>TODO: COPY THE MSG TO A LOCAL VARIABLE
    turtlePose_g(0) = msg->x;
    turtlePose_g(1) = msg->y;
    turtlePose_g(2) = msg->theta;
    pthread_mutex_unlock( &this->count_mutex );

    //#>>>>TODO:PLOT THE OBTAINED DATA
    //ROS_INFO("turtle_vis_node receives curretn position --> x:%d, y:%d, theta:%d", turtlePose_g(0), turtlePose_g(1), turtlePose_g(2));
    ROS_INFO_STREAM("turtle_control_node receives current position --> x: " << turtlePose_desired_g(0) << " y: " << turtlePose_desired_g(1) << " theta: " << turtlePose_desired_g(2));

}

bool TurtleClass::getDPose(turtle_vis::send_desired_pose::Request &req, turtle_vis::send_desired_pose::Response &res)
{
    pthread_mutex_lock( &this->count_mutex );
    //#>>>>TODO:COPY THE REQUEST MSG TO A LOCAL VARIABLE
    turtlePose_desired_g(0) = req.x;
    turtlePose_desired_g(1) = req.y;
    turtlePose_desired_g(2) = req.theta;
    pthread_mutex_unlock( &this->count_mutex );

    //#>>>>TODO:PLOT THE OBTAINED DATA
    // ROS_INFO("turtle_control_node receives desired position --> x:%d, y:%d, theta:%d", turtlePose_desired_g(0), turtlePose_desired_g(1), turtlePose_desired_g(2));
    ROS_INFO_STREAM("turtle_control_node receives desired position --> x: " << turtlePose_desired_g(0) << " y: " << turtlePose_desired_g(1) << " theta: " << turtlePose_desired_g(2));
    res.reply=1;

    return true;
}

Vector3d TurtleClass::getLocalPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}

Vector3d TurtleClass::getLocalDesiredPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_desired_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}




}
