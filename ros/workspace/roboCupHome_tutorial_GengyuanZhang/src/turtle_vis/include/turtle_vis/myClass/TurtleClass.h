#ifndef TURTLECLASS_H
#define TURTLECLASS_H

/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <turtle_vis/DesiredPose.h>
#include <turtle_vis/send_desired_pose.h>

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace Eigen;

namespace turtleSpace
{
    class TurtleClass
    {
    public:

				pthread_mutex_t count_mutex;
        
        TurtleClass();
        ~TurtleClass();

				//#>>>>TODO:CREATE A CALLBACK FUNCTION FOR THE TOPIC turtle_vis::DesiredPose (SEE TurtleClass.cpp)
			
				//#>>>>TODO:CREATE A CALLBACK FUNCTION FOR THE SERVICE turtle_vis::send_desired_pose

				//#>>>>TODO:CREATE 2 METHODS TO GET THE DESIRED TURTLE POSE AND CURRENT TURTLE POSE 
    };


}

#endif // TURTLECLASS_H
