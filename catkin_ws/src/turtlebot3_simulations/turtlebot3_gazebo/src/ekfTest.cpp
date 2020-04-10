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
    int numComponents;

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
    numComponents(2),
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
        turtle_odom = n.subscribe("/odom", 10, &TurtleEkf::cbOdom, this);  // turtle_odom = n.subscribe("/odom", 10, cbOdom);
        turtle_lidar = n.subscribe("/scan", 10, &TurtleEkf::cbLidar, this);  // turtle_lidar = n.subscribe("/scan", 10, cbLidar);

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

    void motionModel(double angVel, double linVel, double deltaT)
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
        std::cout << "Predicted states = " << std::endl;
        std::cout << states(0) << ", " << states(1) << ", " << states(2) << std::endl;
        // std::cout << "X', Y', Th': " << states(0) << ", " << states(1) << ", " << states(2) << std::endl;

        // return states;

    };

    void motionModelVariance(double angVel, double linVel, double deltaT)
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
        std::cout << "Predicted variances = " << std::endl;
        std::cout << variances << std::endl;

        // return variances;

    };

    // static std::vector<double> lidarRangeHeading(const std::vector<float> &lidarRange, const double angleInc)
    std::vector<double> lidarRangeHeading(const std::vector<float> &lidarRange, const double angleInc)
    {
        double avgRange = 0;
        int ctr = 0;
        for (int i = 0; i < lidarRange.size(); ++i)
        {
            if (lidarRange[i] < INF)
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
            if(lidarRange[i] >= INF && lidarRange[i+1] < INF)
            {
                headingStart = (i+1)*angleInc;
                break;
            }
        }
        headingStart = normalizeAngle(headingStart);

        for (int i = 0; i < lidarRange.size()-1; ++i)
        {
            if(lidarRange[i] < INF && lidarRange[i+1] >= INF)
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

        // std::cout << "LIDARLIDARLIDAR EKFEKFEKFEKFEKF" << std::endl;
        // std::cout << "StartAngle, StopAngle, MidAngle: " << headingStart << ", " << headingStop << ", " << headingMiddle << std::endl;
        // std::cout << "Range, Angle: " << avgRange << ", " << headingMiddle << std::endl;

        std::vector<double> output = {avgRange, headingMiddle};
        return output;
    }


    // Also behaves as the sensorModel()
    //OR instead of making it static, might be able to use: 
    //https://answers.ros.org/question/282259/ros-class-with-callback-methods/
    //turtle_lidar = n.subscribe("/scan", 10, &TurtleEkf::cbLidar, this);  --- In the constructor where the subscriber is initiated.

    // static void cbLidar(const sensor_msgs::LaserScan::ConstPtr &msg)
    void cbLidar(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        double angleMin = msg->angle_min;
        double angleInc = msg->angle_increment;
        std::vector<float> lidarRange = msg->ranges;

        std::vector<double> landmarkMeasurement = lidarRangeHeading(lidarRange, angleInc);
        double avgRange = landmarkMeasurement[0];
        double headingMiddle = landmarkMeasurement[1];

        //?? May not be needed to make a copy.
        Eigen::VectorXd predictedStates = states;
        Eigen::MatrixXd predictedVariances = variances;

//// Begin For loop for each landmark

        //?? Setting a landmark manually for now. Will need a loop actually
        int landmarkId = 0;
        int stateIdx = numModelStates + landmarkId;
        if ( !bSeenLandmark(landmarkId) ) // If landmark not seen before, set the prior of that landmark to global position of the landmark
        {
            // NOTE: ujx is the state in states that corsp to this j-th landmark

            // u_jx = u_tx + r*cos(phi + u_tth)
            // land_x = robot_x + r*cos(phi + robot_heading)
            states(stateIdx) = predictedStates(0) + avgRange * cos(headingMiddle + predictedStates(2));
            states(stateIdx+1) = predictedStates(1) + avgRange * sin(headingMiddle + predictedStates(2));

            bSeenLandmark(landmarkId) = 1;
        }

        double delx = states(stateIdx) - predictedStates(0);
        double dely = states(stateIdx+1) - predictedStates(1);
        double q = delx*delx + dely*dely;

        Eigen::VectorXd zj(2);
        Eigen::VectorXd zjHat (2);
        zj << avgRange, headingMiddle;
        zjHat << std::sqrt(q) , ( atan2(dely, delx) - predictedStates(2) );
        std::cout << "z and zHat = " << std::endl;
        std::cout << zj << std::endl;
        std::cout << zjHat << std::endl;

        // (numComponentsMotionModel + numComponentsLandmarks) * (numComponentsMotionModel + numComponentsLandmarks*numLandmarks) 
        // (3 + 2) * (3 + 2*numLandmarks)
        //?? Replace all 2s everywhere with numLandmarkComponents OR numLandmarkDims
        Eigen::MatrixXd Fxj = Eigen::MatrixXd::Zero( (numModelStates + numComponents), numTotStates);        
        Fxj.topLeftCorner(numModelStates,numModelStates) = Eigen::MatrixXd::Identity(numModelStates,numModelStates);
        
        Fxj(numModelStates, numModelStates + landmarkId) = 1;
        Fxj(numModelStates+1, numModelStates + landmarkId+1) = 1;
        std::cout << "Fxj = " << std::endl;
        std::cout << Fxj << std::endl;

        // Partial differential of:
        // zHat_x wrt modelX, modelY, modelTh, mx, my
        // zHat_y wrt modelX, modelY, modelTh, mx, my
        // H*z ==nt Cx or Hx in traditional state space model, but here input x for this step is estimated/expected sensor reading = zHat
        Eigen::MatrixXd H (numComponents, numTotStates);
        H << 
            std::sqrt(q)*delx , -std::sqrt(q)*dely , 0    , -std::sqrt(q)*delx   , std::sqrt(q)*dely ,
            dely              , delx               , -1   , -dely                , -delx;
        H = (1/q) * H;
        // std::cout << "H = " << std::endl;
        // std::cout << H << std::endl;

        // (3+2)x(3+2n) = 5x5 for single landmark case, with each landmark being 2D
        Eigen::MatrixXd HFxj (numTotStates, numTotStates);
        HFxj = H * Fxj;
        std::cout << "HFxj = " << std::endl; // Same as H since with only one obstacle, we get just F to be identity
        std::cout << HFxj << std::endl;        

        // K = sigma * H' * (H * sigma * H')
        // (3+2n)x(3+2n) * (5)x(5) * inv( (5)x(5) * ()x() * (5)x(5) )
        // (5)x(5) * (5)x(5) * inv( (5)x(5) * (5)x(5) * (5)x(5) )
        Eigen::MatrixXd K (numTotStates, numComponents);
        K = predictedVariances*HFxj.transpose()*( H * predictedVariances * HFxj.transpose() ).inverse(); //?? Add process/variance noise inside inversion term
        std::cout << "K = " << std::endl;
        std::cout << K << std::endl;

        predictedStates = predictedStates + K * (zj - zjHat);
        predictedVariances = ( Eigen::MatrixXd::Identity(numTotStates, numTotStates) - K*HFxj) * predictedVariances;
        
//// End For loop for each landmark

        states = predictedStates;
        variances = predictedVariances;

        std::cout << "Corrected states and variances" << std::endl;
        std::cout << states << std::endl;
        std::cout << variances << std::endl;

    };

    // OR instead of making it static, can use:
    //https://answers.ros.org/question/282259/ros-class-with-callback-methods/
    //turtle_odom = n.subscribe("/odom", 10, &TurtleEkf::cbOdom, this);  --- In the constructor where the subscriber is initiated.

    // static void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
    void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
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
    ros::init(argc, argv, "ekf_Test");

    TurtleEkf* turtlebot = new TurtleEkf(); // Init on heap so that large lidar data isn't an issue

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        std::cout << "===========" << std::endl;
        std::cout << ">>> Prediction Step" << std::endl;
        turtlebot->controlLoop();
        std::cout << ">>> Correction Step" << std::endl;
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
//?? Get rid of the the static functions in the class by using the "&" template in the subscriber description line