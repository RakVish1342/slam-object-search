#include "/opt/ros/kinetic/include/ros/ros.h"
#include "/opt/ros/kinetic/include/geometry_msgs/Twist.h"
// #include "/opt/ros/kinetic/include/eigen_stl_containers/eigen_stl_containers.h"
#include "Eigen/Dense" // Added include library EIGEN_DIRS=/usr/include/eigen3
#include <math.h>
#include <limits>
#include <nav_msgs/Odometry.h> // Found it using "rostopic info /odom". Is located in /opt/ros/kinetic/include/nav_msgs
#include <sensor_msgs/LaserScan.h> // Found it using "rostopic info /scan". Is located in /opt/ros/kinetic/include/sensor_msgs
#include <vector>


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



class turtleEkf
{

private:
    double nextX, prevX;
    double nextY, prevY;
    double nextTh, prevTh; // Angle measured wrt global horizontal frame that gets initialized to
                                                // Z(into plane), Y(forward direction of init robot pose), X(right of robot)
    float INF; // float type since lidar vals are in float
    int numModelStates;
    int numLandmarks;
    int numStates;
    std::vector<double> states;
    Eigen::MatrixXd variances;

public:
    turtleEkf() :
    INF(std::numeric_limits<float>::max()),
    nextX(0), prevX(0),
    nextY(0), prevY(0),
    nextTh(PI/2.0), prevTh(PI/2.0),
    numModelStates(3),
    numLandmarks(1),
    numStates(numModelStates + 2*numLandmarks),
    states(std::vector<double> (0)), 
    variances(Eigen::MatrixXd::Zero(numStates, numStates))
    {
        // Set landmark variances to inf
        variances.bottomRightCorner(numLandmarks, numLandmarks) = Eigen::MatrixXd::Constant(numLandmarks, numLandmarks, INF);
    };

    ~turtleEkf() {};

    // void motionModel()
    std::vector<double> motionModel(double angVel, double linVel, double deltaT)
    {
    
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
            // std::cout << "ELSE" << std::endl;
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

        return {nextX, nextY, nextTh};

    };

    std::vector<double> motionModelFx(double angVel, double linVel, double deltaT)
    {


        return {nextX, nextY, nextTh};

    };

};


void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{

    double odomX = msg->pose.pose.position.x;
    double odomY = msg->pose.pose.position.y;
    std::cout << "odomX, odomY: " << odomX << ", " << odomY << std::endl;
       
}

void cbLidarTest(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    double angleMin = msg->angle_min;
    double angleInc = msg->angle_increment;

    // std::vector<double> lidarRange = msg->ranges;
    std::vector<float> lidarRange = msg->ranges;
    // std::vector<double> lidarIntensity = msg.intensities;

    std::cout << "--- LIDAR ---" << std::endl;
    for (float ray : lidarRange)
    {

        // Considering only finite ranges
        if (ray >= std::numeric_limits<float>::max())
        {
            std::cout << "LEL";
            continue;
        }
        std::cout << ray << ", ";
    }
    std::cout << std::endl;
    std::cout << "--- /LIDAR ---" << std::endl;    

}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "ekf_singleBlock");
    ROS_INFO_STREAM("Started: efk_singleBlock Node");

    ros::NodeHandle n;
    ros::Publisher turtle_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber turtle_odom = n.subscribe("/odom", 10, cbOdom);
    ros::Subscriber turtle_lidar = n.subscribe("/scan", 10, cbLidarTest);

    double tstart = ros::Time::now().toSec();
    double currT = tstart;
    double prevT = tstart;

    geometry_msgs::Twist msg;

    turtleEkf* turtlebot = new turtleEkf(); // Init on heap so that large lidar data isn't an issue

    ros::Rate loop_rate(100);
    while (ros::ok()) 
    {
        // global time
        double tstop = ros::Time::now().toSec() - tstart;

        double linVel, angVel;
        if(tstop>0 && tstop<8)
        {
            msg.linear.x = 0.5;
            msg.angular.z = 0.25;
            linVel = 0.5;
            angVel = 0.25;
        }
        else if(tstop>=8)
        {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            linVel = 0.0;
            angVel = 0.0;         
        }

        // send velocity commands
        turtle_vel.publish(msg);

        //delta_t calc
        currT = ros::Time::now().toSec();
        double deltaT = currT - prevT;
        prevT = currT;

        std::cout << "---" << std::endl;
        std::cout << "rad, time: " << linVel/angVel << ", " << deltaT << std::endl;

        // Call the motion model and pass lin and angular vel
        std::vector<double> xyth = turtlebot->motionModel(angVel, linVel, deltaT);
        // turtlebot->motionModel();

        // Call subscriber callbacks once
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}