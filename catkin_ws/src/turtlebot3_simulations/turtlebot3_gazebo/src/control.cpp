#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include "/opt/ros/kinetic/include/eigen_stl_containers/eigen_stl_containers.h"
#include <aruco_msgs/MarkerArray.h> // Located in devel/include/aruco_msgs/MarkerArray.h, not sure how it gets generated automatically or if it gets shifted or copied automatically
// #include "aruco.h" // Fix CMakeFiles.txt so that the aruco_ros and aruco_msgs package and msgs get discovered properly like nav_msgs etc
#include "Eigen/Dense" // Added include library EIGEN_DIRS=/usr/include/eigen3
#include <math.h>
#include <limits>
#include <nav_msgs/Odometry.h> // Found it using "rostopic info /odom". Is located in /opt/ros/kinetic/include/nav_msgs
#include <sensor_msgs/LaserScan.h> // Found it using "rostopic info /scan". Is located in /opt/ros/kinetic/include/sensor_msgs


class ControlLoop
{

public:
    ros::NodeHandle n;
    ros::Publisher control_vel;
    double globalTStart;
    double currT;
    double prevT;
    double deltaT;
    double timeThresh;

    ControlLoop():
    timeThresh(6),
    globalTStart(ros::Time::now().toSec())
    {
        ROS_INFO("Started Node: controlLoop");
        ROS_INFO_STREAM("Started Node: controlLoop");

        control_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    };

    void control()
    {
        std::cout << "--------------" << std::endl;
        geometry_msgs::Twist msg;        

        // global execution time
        double globalTStop = ros::Time::now().toSec() - globalTStart;
        double timeWaitGazebo = 10; // Wait 4s for gazebo to initialize

        double transitionTime = 0;
        double speed = 0.25;

        // double linVel, angVel;
        // // if(globalTStop > 0 && globalTStop < timeThresh)
        // if(globalTStop > timeWaitGazebo && globalTStop < timeWaitGazebo + timeThresh)
        // {
        //     std::cout << ">>> MOVING1..." << globalTStop << std::endl;
        //     msg.linear.x = speed;
        //     msg.angular.z = -speed;

        // } 
        // else if(globalTStop >= (timeWaitGazebo + timeThresh))
        // {
        //     std::cout << ">>> STOPPED..." << globalTStop << std::endl;
        //     msg.linear.x = 0.0;
        //     msg.angular.z = 0.0;    
        // }


        double linVel, angVel;
        // if(globalTStop > 0 && globalTStop < timeThresh)
        if(globalTStop > timeWaitGazebo && globalTStop < timeWaitGazebo + timeThresh)
        {
            std::cout << ">>> MOVING1..." << globalTStop << std::endl;
            msg.linear.x = speed;
            msg.angular.z = speed;

        }

        else if(globalTStop > timeWaitGazebo + timeThresh && globalTStop < (timeWaitGazebo + timeThresh + transitionTime) )
        {
            std::cout << ">>> TRANSITION1..." << globalTStop << std::endl;
            msg.linear.x = 0;
            msg.angular.z = 0;

        }

        else if( (globalTStop > timeWaitGazebo + timeThresh + transitionTime) && globalTStop < (timeWaitGazebo + timeThresh + 2*timeThresh + 1 ) )
        {
            std::cout << ">>> MOVING2..." << globalTStop << std::endl;
            msg.linear.x = speed;
            msg.angular.z = -speed;

        }

        else if( globalTStop > (timeWaitGazebo + timeThresh + 2*timeThresh + 1) && globalTStop < (timeWaitGazebo + timeThresh + 3*timeThresh + 1  ) )
        {
            std::cout << ">>> MOVING3..." << globalTStop << std::endl;
            msg.linear.x = speed/2;
            msg.angular.z = 0;

        }

        else if(globalTStop >= (timeWaitGazebo + 3*timeThresh + 1))
        {
            std::cout << ">>> STOPPED..." << globalTStop << std::endl;
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;    
        }

        // send velocity commands
        control_vel.publish(msg);

        //delta_t calc
        double currT = ros::Time::now().toSec();
        double deltaT = currT - prevT;
        prevT = currT;
        // std::cout << globalTStart << std::endl;
        // std::cout << currT << std::endl;
        // std::cout << deltaT << std::endl;
    };

    double getDeltaT()
    {
        return this->deltaT;
    }

};

int main(int argc, char** argv)
{
    // Must perform ROS init before object creation, as node handles are created in the obj constructor
    ros::init(argc, argv, "control_loop");

    ControlLoop* control = new ControlLoop(); // Init on heap so that large lidar data isn't an issue

    ros::Rate loop_rate(100);
    int i = 0;
    while(ros::ok())
    {
        std::cout << "===========" << std::endl;
        std::cout << ++i << std::endl;

        // std::cout << control->getDeltaT() << std::endl;
        // std::cout << control->currT << std::endl;
        // std::cout << control->deltaT << std::endl;


        control->control();


        loop_rate.sleep();
    }

    return 0;
}