#include<turtle_vis/myClass/TurtleClass.h>

namespace turtleSpace {

TurtleClass::TurtleClass()
{
    //#>>>>TODO: INITIALIZE MEMBER VARIABLES
    count_mutex = PTHREAD_MUTEX_INITIALIZER;
}
TurtleClass::~TurtleClass()
{

}

void TurtleClass::getPose(const turtle_vis::DesiredPose::ConstPtr &msg)
{
    pthread_mutex_lock( &this->count_mutex );
    //#>>>>TODO: COPY THE MSG TO A LOCAL VARIABLE
    pthread_mutex_unlock( &this->count_mutex );

    //#>>>>TODO:PLOT THE OBTAINED DATA
}

bool TurtleClass::getDPose(turtle_vis::send_desired_pose::Request &req, turtle_vis::send_desired_pose::Response &res)
{
    pthread_mutex_lock( &this->count_mutex );
    //#>>>>TODO:COPY THE REQUEST MSG TO A LOCAL VARIABLE
    pthread_mutex_unlock( &this->count_mutex );

    //#>>>>TODO:PLOT THE OBTAINED DATA

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
