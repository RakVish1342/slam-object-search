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
    float INF; // float type since lidar vals are in float

    int numModelStates;
    int numLandmarks;
    int numStates;

    Eigen::VectorXd states;
    Eigen::VectorXd prevStates;
    Eigen::MatrixXd variances;
    Eigen::MatrixXd Fx;

public:
    turtleEkf() :
    INF(std::numeric_limits<float>::max()),
    numModelStates(3),
    numLandmarks(1),
    numStates(numModelStates + 2*numLandmarks),
    states(Eigen::VectorXd::Zero(numStates)),
    prevStates(Eigen::VectorXd::Zero(numStates)),
    variances(Eigen::MatrixXd::Zero(numStates, numStates)),
    Fx (Eigen::MatrixXd::Zero(numStates, numStates))
    {
        // Init theta to PI/2 as per X axis definition: perp to the right
        states(2) = PI/2.0;
        prevStates(2) = PI/2.0;
        // Set landmark variances to inf
        variances.bottomRightCorner(2*numLandmarks, 2*numLandmarks) = Eigen::MatrixXd::Constant(2*numLandmarks, 2*numLandmarks, INF);
        Fx.topLeftCorner(numModelStates, numModelStates) = Eigen::MatrixXd::Identity(numModelStates, numModelStates);
    };

    ~turtleEkf() {};

    Eigen::VectorXd motionModel(double angVel, double linVel, double deltaT)
    {
    
        if (angVel > 0.001)
        {
            std::cout << "IF" << std::endl;
            double r = linVel/angVel;

            //?? ADD other condition of angles 
            states(0) = prevStates(0) + ( -r*sin(prevStates(2)) + r*sin(prevStates(2) + angVel*deltaT) );
            states(1) = prevStates(1) + ( +r*cos(prevStates(2)) - r*cos(prevStates(2) + angVel*deltaT) );
            states(2) = prevStates(2) + angVel*deltaT;
        }
        else
        {
            // std::cout << "ELSE" << std::endl;
            double dist = linVel*deltaT;

            // ADD other condition of angles 
            states(0) = prevStates(0) - dist*cos(prevStates(2));
            states(1) = prevStates(1) + dist*sin(prevStates(2));
            states(2) = prevStates(2);
        }

        states(2) = normalizeAngle(states(2));

        prevStates(0) = states(0);
        prevStates(1) = states(1);
        prevStates(2) = states(2);

        std::cout << "X', Y', Th': " << states(0) << ", " << states(1) << ", " << states(2) << std::endl;

        return states;

    };

    // Eigen::VectorXd motionModelFx(double angVel, double linVel, double deltaT)
    // {
    //     states = Fx*states;
    //     return states;
    // };

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
        turtlebot->motionModel(angVel, linVel, deltaT);
        // turtlebot->motionModel();

        // Call subscriber callbacks once
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}