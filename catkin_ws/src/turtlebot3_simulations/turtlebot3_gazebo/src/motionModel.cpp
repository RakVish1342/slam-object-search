#include "/opt/ros/kinetic/include/ros/ros.h"
#include "/opt/ros/kinetic/include/geometry_msgs/Twist.h"
#include <math.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159265


double normalizeAngle(double angle)
{
	double output;
	//double rem = std::abs(angle) % PI;
	double rem = std::fmod(std::abs(angle), PI);
	double quo = (std::abs(angle) - rem) / PI;

	int oddQuo = std::fmod(quo, 2);
	// Positive angle input or 0
	if (angle >= 0)
	{
		if (oddQuo == 0)
		{
			output = rem;
		}
		else
		{
			output = -(PI - rem);
		}
	}
	// Negative Angle received
	else
	{
		if (oddQuo == 0)
		{
			output = -rem;
		}
		else
		{
			output = (PI - rem);
		}
	}

	return output;
}


void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{

    double odomX = msg->pose.pose.position.x;
    double odomY = msg->pose.pose.position.y;
    std::cout << "odomX, odomY: " << odomX << ", " << odomY << std::endl;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "motion_model");
    ROS_INFO_STREAM("Started: motion_model Node");

    ros::NodeHandle n;

    ros::Publisher turtle_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // ros::Subscriber turtle_odom = n.subscribe<geometry_msgs::Twist>("/odom", 10, cbOdom);
    ros::Subscriber turtle_odom = n.subscribe("/odom", 10, cbOdom);

    double tstart = ros::Time::now().toSec();
    double currT = tstart;
    double prevT = tstart;

    double nextX = 0, prevX = 0;
    double nextY = 0, prevY = 0;
    double nextTh = PI/2.0, prevTh = PI/2.0; // Angle measured wrt global horizontal frame that gets initialized to
                                             // Z(into plane), Y(forward direction of init robot pose), X(right of robot)

    geometry_msgs::Twist msg;

    ros::Rate loop_rate(100);
    while (ros::ok()) 
    {
        // global time
        double tstop = ros::Time::now().toSec() - tstart;
        std::cout << "global_time: " << tstop << std::endl;

        //delta_t calc
        currT = ros::Time::now().toSec();
        double deltaT = currT - prevT;
        prevT = currT;
        std::cout << std::endl;
        std::cout << "DeltaT: " << deltaT << std::endl;

        double linVel, angVel;
        // test cases
        // t: 0,4; vel: 0.5,0.25 (linVel, angVel)
        // t: 0,6; vel: 0.5,0.25 (linVel, angVel)
        // t: 0,6; vel: 0.25,0.5 (linVel, angVel)
        // t: 0,8; vel: 0.25,0.5 (linVel, angVel)

        if(tstop>0 && tstop<8)
        {
            msg.linear.x = 0.5;
            msg.angular.z = 0.25;
            linVel = 0.5;
            angVel = 0.25;
        }
        // else if(tstop>=3 && tstop<4)
        // {
        //     msg.linear.x = 0.5;
        //     msg.angular.z = PI/2;
        //     linVel = 0.5;
        //     angVel = 3.1415/2;
        // }
        else if(tstop>=8)
        {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            linVel = 0.0;
            angVel = 0.0;         
        }

        
        //motion model
        if (angVel > 0.001)
        {
            std::cout << "IF" << std::endl;
            double r = linVel/angVel;

            //?? ADD other condition of angles 
            nextX = prevX + ( -r*sin(prevTh) + r*sin(prevTh + angVel*deltaT) );
            nextY = prevY + ( +r*cos(prevTh) - r*cos(prevTh + angVel*deltaT) );
            nextTh = prevTh + angVel*deltaT;
        }
        else
        {
            std::cout << "ELSE" << std::endl;
            double dist = linVel*deltaT;

            // ADD other condition of angles 
            nextX = prevX - dist*cos(prevTh);
            nextY = prevY + dist*sin(prevTh);
            nextTh = prevTh;
        }

        nextTh = normalizeAngle(nextTh);
    
        prevX = nextX;
        prevY = nextY;
        prevTh = nextTh;

        std::cout << "X', Y', Th': " << nextX << ", " << nextY << ", " << nextTh << std::endl;

        turtle_vel.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}