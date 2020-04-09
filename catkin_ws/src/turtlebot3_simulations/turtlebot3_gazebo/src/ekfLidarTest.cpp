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
#define STATIC_INF std::numeric_limits<float>::max()



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



class TurtleEkf
{

private:
    ros::NodeHandle n;
    ros::Publisher turtle_vel;
    ros::Subscriber turtle_odom;
    ros::Subscriber turtle_lidar;
    double globalTStart;
    double prevT;

    float INF; // float type since lidar vals are in float

    int numModelStates;
    int numLandmarks;
    int numTotStates;

    Eigen::VectorXd states;
    Eigen::VectorXd prevStates; // Prev State vector also being maintained since while calc of variance, prevState vec would be reqd, 
                                // but the states would have already been updated as per motion eqns
    Eigen::MatrixXd variances;
    Eigen::MatrixXd Fx;
    Eigen::VectorXd bSeenLandmark;


public:
    TurtleEkf() :
    INF(std::numeric_limits<float>::max()),
    numModelStates(3),
    numLandmarks(1),
    numTotStates(numModelStates + 2*numLandmarks),
    states(Eigen::VectorXd::Zero(numTotStates)),
    prevStates(Eigen::VectorXd::Zero(numTotStates)),
    variances(Eigen::MatrixXd::Zero(numTotStates, numTotStates)),
    Fx (Eigen::MatrixXd::Zero(numModelStates, numTotStates)), 
    bSeenLandmark(Eigen::VectorXd::Zero(numLandmarks))
    {
        ROS_INFO("Started Node: efk_singleBlock");
        ROS_INFO_STREAM("Started Node: efk_singleBlock");

        // Init the publishers and subscribers
        turtle_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        turtle_odom = n.subscribe("/odom", 10, cbOdom);
        turtle_lidar = n.subscribe("/scan", 10, cbLidar);

        // Init theta to PI/2 as per X axis definition: perp to the right
        states(2) = PI/2.0;
        prevStates(2) = PI/2.0;

        // Set landmark variances to inf
        variances.bottomRightCorner(2*numLandmarks, 2*numLandmarks) = Eigen::MatrixXd::Constant(2*numLandmarks, 2*numLandmarks, INF);
        variances.topRightCorner(numModelStates, 2*numLandmarks) = Eigen::MatrixXd::Constant(numModelStates, 2*numLandmarks, INF);
        variances.bottomLeftCorner(2*numLandmarks, numModelStates) = Eigen::MatrixXd::Constant(2*numLandmarks, numModelStates, INF);
        Fx.topLeftCorner(numModelStates, numModelStates) = Eigen::MatrixXd::Identity(numModelStates, numModelStates);

        globalTStart = ros::Time::now().toSec();
        prevT = globalTStart;

    };

    ~TurtleEkf() {};

    Eigen::VectorXd motionModel(double angVel, double linVel, double deltaT)
    {
    
        if (angVel > 0.001)
        {
            double r = linVel/angVel;

            //?? ADD other condition of angles 
            states(0) = prevStates(0) + ( -r*sin(prevStates(2)) + r*sin(prevStates(2) + angVel*deltaT) );
            states(1) = prevStates(1) + ( +r*cos(prevStates(2)) - r*cos(prevStates(2) + angVel*deltaT) );
            states(2) = prevStates(2) + angVel*deltaT;
        }
        else
        {
            double dist = linVel*deltaT;

            // ADD other condition of angles 
            states(0) = prevStates(0) - dist*cos(prevStates(2));
            states(1) = prevStates(1) + dist*sin(prevStates(2));
            states(2) = prevStates(2);
        }

        states(2) = normalizeAngle(states(2));

        //?? Will need to remove this update step from here. Take it to the end of sensor model
        prevStates = states; 

        //??prt
        // std::cout << "X', Y', Th': " << states(0) << ", " << states(1) << ", " << states(2) << std::endl;

        return states;

    };

    Eigen::VectorXd motionModelVariance(double angVel, double linVel, double deltaT)
    {
        double derivXTh, derivYTh;
    
        if (angVel > 0.001)
        {
            double r = linVel/angVel;

            // Derivative of X and Y wrt Th
            derivXTh = -r*cos(prevStates(2)) + r*cos(prevStates(2) + angVel*deltaT);
            derivYTh = -r*sin(prevStates(2)) + r*sin(prevStates(2) + angVel*deltaT);
            // derivTh = 1 got from Identity addition
        }
        else
        {
            double dist = linVel*deltaT;

            // ADD other condition of angles 
            derivXTh = dist*sin(prevStates(2));
            derivYTh = dist*cos(prevStates(2));
            // derivTh = 1 got from Identity addition
        }

        Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(numModelStates, numModelStates);
        tmp(0,2) = derivXTh;
        tmp(1,2) = derivYTh;
        // Jacobian of non-linear motion model
        Eigen::MatrixXd Gt =  Eigen::MatrixXd::Identity(numTotStates, numTotStates) + Fx.transpose() * tmp * Fx;

        // Variance Calculation
        variances = Gt * variances * Gt.transpose(); //?? Add process/gaussian noise

        //??prt
        // std::cout << tmp << std::endl;
        // std::cout << "..." << std::endl;
        // std::cout << variances << std::endl;

        return states;

    };

    // Also behaves as the sensorModel()
    static void cbLidar(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        double angleMin = msg->angle_min;
        double angleInc = msg->angle_increment;

        // std::vector<double> lidarRange = msg->ranges;
        std::vector<float> lidarRange = msg->ranges;
        // std::vector<double> lidarIntensity = msg.intensities;

        // std::cout << "--- LIDAR ---" << std::endl;
        // for (float ray : lidarRange)
        // {

        //     // Considering only finite ranges
        //     if (ray >= std::numeric_limits<float>::max())
        //     {
        //         std::cout << "LEL";
        //         continue;
        //     }
        //     std::cout << ray << ", ";
        // }
        // std::cout << std::endl;
        // std::cout << "--- /LIDAR ---" << std::endl;

        //?? Assuming all lidar points belong to this one landmark
        // So taking avg of the range and heading of these lidar points, for this single landmark
        double avgRange = 0;
        int ctr = 0;
        for (int i = 0; i < lidarRange.size(); ++i)
        {
            if (lidarRange[i] < STATIC_INF)
            {
                avgRange += lidarRange[i];
                ctr += 1;
            }
        }
        avgRange /= ctr;

        double headingStart = 0;
        double headingStop = 0;
        double headingMiddle = 0;
        for (int i = 0; i < lidarRange.size()-1; ++i)
        {
            if(lidarRange[i] >= STATIC_INF && lidarRange[i+1] < STATIC_INF)
            {
                headingStart = (i+1)*angleInc;
                break;
            }
        }
        headingStart = normalizeAngle(headingStart);

        for (int i = 0; i < lidarRange.size()-1; ++i)
        {
            if(lidarRange[i] < STATIC_INF && lidarRange[i+1] >= STATIC_INF)
            {
                headingStop = i*angleInc;
            }
        }        
        headingStop = normalizeAngle(headingStop);

        // Heading points are in the positive half. Take avg and assign +ve sign
        if ( (headingStart > 0) && (headingStart <= PI) && (headingStop > 0) && (headingStop <= PI) )
        {
            headingMiddle = std::abs(headingStop + headingStart) / 2.0;
        }
        // Heading points are in the negative half. Take avg and assign -ve sign
        else if ( (headingStart < 0) && (headingStart >= -PI) && (headingStop < 0) && (headingStop >= -PI) )
        {
            headingMiddle = - std::abs(headingStop + headingStart) / 2.0;
        }
        // Start point is in positive half and stop is in the negative half. 
        // Find actual measurement angle viz complement of direct angle. Half of this angle measurement to be added to start and normalized
        else if ( (headingStart > 0) && (headingStart <= PI) && (headingStop < 0) && (headingStop >= -PI) )
        {
            double measurementAngle  = 2*PI - (std::abs(headingStart) + std::abs(headingStop));
            headingMiddle = normalizeAngle( headingStart + measurementAngle/2.0 );
        }
        // Start point is in negative half and stop point is in the positive half
        // Find angle range in between the start and stop points. Assign sign based on which is larger in magnitude
        else if ( (headingStart < 0) && (headingStart >= -PI) && (headingStop > 0) && (headingStop <= PI) )
        {
            double measurementAngle  = std::abs(headingStart) + std::abs(headingStop);
            if ( std::abs(headingStop) >= std::abs(headingStart) ) // Will fall on positive side
            {
                headingMiddle = + ( measurementAngle/2.0 - std::abs(headingStart) );
            }
            else // Will fall on negative side
            {
                headingMiddle = - ( measurementAngle/2.0 - std::abs(headingStop) );
            }
        }


        std::cout << "LIDARLIDARLIDAR" << std::endl;
        std::cout << "StartAngle, StopAngle, MidAngle: " << headingStart << ", " << headingStop << ", " << headingMiddle << std::endl;
        std::cout << "Range, Angle: " << avgRange << ", " << headingMiddle << std::endl;

        // Eigen::VectorXd predictedStates = states; //?? May not be needed to make a copy.

        // landmarkId = 1; // Setting it manuall this time
        // if (!bSeenLandmark(landmarkId))
        // {
            
        // }
        // else
        // {

        // }



    };

    static void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
    {

        double odomX = msg->pose.pose.position.x;
        double odomY = msg->pose.pose.position.y;

        //??prt        
        // std::cout << "odomX, odomY: " << odomX << ", " << odomY << std::endl;
        
    };

    void controlLoop()
    {
        geometry_msgs::Twist msg;        

        // global execution time
        double globalTStop = ros::Time::now().toSec() - globalTStart;

        double linVel, angVel;
        if(globalTStop > 0 && globalTStop < 0.001)
        {
            msg.linear.x = 0.5;
            msg.angular.z = 0.25;
            linVel = 0.5;
            angVel = 0.25;
        }
        else if(globalTStop >= 0.001)
        {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            linVel = 0.0;
            angVel = 0.0;         
        }

        // send velocity commands
        turtle_vel.publish(msg);

        //delta_t calc
        double currT = ros::Time::now().toSec();
        double deltaT = currT - prevT;
        prevT = currT;

        //??prt
        // std::cout << "---" << std::endl;
        // std::cout << "rad, time: " << linVel/angVel << ", " << deltaT << std::endl;

        // Call the motion model and pass lin and angular vel
        motionModel(angVel, linVel, deltaT);
        motionModelVariance(angVel, linVel, deltaT);

    };

};


int main(int argc, char** argv)
{
    // Must perform ROS init before object creation, as node handles are created in the obj constructor
    ros::init(argc, argv, "ekf_singleBlock");

    TurtleEkf* turtlebot = new TurtleEkf(); // Init on heap so that large lidar data isn't an issue

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        turtlebot->controlLoop();
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    return 0;
}


//?? Other cases/components of angles to be considered in motion model?...esp when bot goaes beyond +- 180deg?
//?? Add noise to variance eqn and motion model?
//?? When sim ends and v and omega zero, what dies motion model output?? Should I kill sim little before the end and check vals
//?? For pure motion model only, check eqn wise why SIGMA ain't changing ... does makes sense since no info about landmarks so that
// remains at inf, and only motion model is present so that stays fully certain at 0. Eqn wise, tmp happens to just stay really close to 0
// Later see how it performs when some noise is incorporated. Noise to be added to v, w, th and to variance matrix as Rt.