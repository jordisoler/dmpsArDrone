#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "dynamic_reconfigure/server.h"
#include "prj_drone/dynamic_paramsConfig.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "ardrone_autonomy/Navdata.h"
#include <cstdlib>
#include <ctime>
#include "ar_pose/ARMarkers.h"
#include "tf/transform_listener.h"


// States of the ArDrone
#define UNKNOWN 0
#define INITIED 1
#define LANDED 2
#define FLYING 3
#define HOVERING 4
#define TEST 5
#define TAKINGOFF 6
#define FLYING2 7
#define LANDING 8
#define LOOPING 9

// Duration of some events
const static ros::Duration TIME_TURNING_ = ros::Duration(6);
const static ros::Duration TIME_WAITING_ = ros::Duration(2);
const static ros::Duration TIME_WAITING2_ = ros::Duration(2);

// Sctructure containing actions to be done and desired velocities
struct ActionsToDo
{
	bool update_takeoff;
	bool update_land;
	bool update_toggle_cam;
	bool update_reset;
	double update_velocityX;
	double update_velocityY;
	double update_velocityZ;
	double update_angularX;
	double update_angularY;
	double update_angularZ;

	ActionsToDo(){
		update_takeoff=false;
		update_land=false;
		update_toggle_cam=false;
		update_reset=false;
		update_velocityX=false;
		update_velocityY=false;
		update_velocityZ=false;
		update_angularX=false;
		update_angularY=false;
		update_angularZ=false;
	}
};

// Global  variables
ActionsToDo actions_todo;

int RECEIVED_STATE_, CURRENT_STATE_;

bool cam_, turning_, waiting_, performing_, waiting2_, ready2takeoff_;	// Indicates whether the bottom cam is  active or not.

double _z_, _y_, _x_, yaw_;
double desired_distance_z_=1, desired_distance_y_=0, desired_distance_x_=0, desired_angle_yaw_=1.57;
double final_distance_z_=1, final_distance_y_=0, final_distance_x_=2;

ros::Time time_, prev_time_, t0_;

/**************************************************
*	AUXILLIARY FUNCTIONS
**************************************************/
/*
 *Toggle the camera tacking into account the flags cam_ and turning_
 */
void tgcam(ros::ServiceClient camaras_sc)
{
	std_srvs::Empty msgT;
	camaras_sc.call(msgT);
	ROS_INFO("Toggle camera!");
	cam_ = !cam_;
	turning_ = turning_ && cam_;
}

/*
 *Get current pose from the markers (assuming miniproject_world)
 */
void inverseKinematics(double mx, double my, double mz)
{
    if(cam_){
	_x_ = mx;
	_y_ = my;
	_z_ = mz;
    }else{
	_x_ = 3-mz;
	_y_ = mx;
	_z_ = 1+my;
	ROS_INFO("Pose: x=%f, y=%f, z=%f", _x_,_y_,_z_);
	//ROS_INFO("Pose: x=%f, y=%f, z=%f", mx,my,mz);
    }
}

/*
 *Trajectory via point generator (Trajectory 1, straight line)
 */
geometry_msgs::Point traj1()
{
    double dn = ros::Duration(ros::Time::now()-t0_).toSec();
    double trajtime = 3;
    double initPose [] = {0, 0, 1};
    double finalPose [] = {2, 0, 1};
    double currentPose [3];
    geometry_msgs::Point pr;

    for(int i=0; i<3; i++){
	currentPose[i] = (finalPose[i]-initPose[i])/trajtime;
    }

    pr.x = currentPose[1];
    pr.y = currentPose[2];
    pr.z = currentPose[3];
    return pr;
}


/**************************************************
*	CALLBACKS
**************************************************/
void dynrec_callback(prj_drone::dynamic_paramsConfig &config, uint32_t level)
{
	// Handling dynamic reconfigure changes
	// Actions
	actions_todo.update_takeoff=config.take_off;
	config.take_off=false;
	actions_todo.update_land=config.land;
	config.land=false;
	actions_todo.update_toggle_cam=config.cambio_camara;
	config.cambio_camara=false;
	actions_todo.update_reset=config.reset;
	config.reset=false;
	actions_todo.update_velocityX=config.velocidadX;
	actions_todo.update_velocityY=config.velocidadY;
	actions_todo.update_velocityZ=config.velocidadZ;
	actions_todo.update_angularX=config.angularX;
	actions_todo.update_angularY=config.angularY;
	actions_todo.update_angularZ=config.angularZ;

	// Desired distances
	desired_distance_z_=config.z_desitjada;
	desired_distance_y_=config.y_desitjada;
	desired_distance_x_=config.x_desitjada;

	ROS_INFO("Reconfigure Request. X: %f, Y: %f, Z: %f", desired_distance_x_, desired_distance_y_, desired_distance_z_);
}

void MarkerCallback(const ar_pose::ARMarkers::ConstPtr& message)
{
	// Handling marker data
	if  (message->markers.size()>0)
	{
		double roll, pitch;
		double marker_z_=message->markers[0].pose.pose.position.z;
		double marker_y_=message->markers[0].pose.pose.position.y;
		double marker_x_=message->markers[0].pose.pose.position.x;

		tf::Quaternion qt;
		tf::quaternionMsgToTF(message->markers[0].pose.pose.orientation,qt);
		tf::Matrix3x3(qt).getRPY(roll, pitch, yaw_);

		inverseKinematics(marker_x_,marker_y_,marker_z_);
	}
}

void NavdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg_Navdata)
{
	// Getting ArDrone state
	RECEIVED_STATE_ = msg_Navdata->state;
}

void callbackTimer(const ros::TimerEvent& tEvent)
{
	// Timer callback
	time_ = tEvent.last_real;
}

/**************************************************
*	MAIN FUNCTION
**************************************************/
int main(int argc, char **argv)
{
	// Set up ROS node
	ros::init(argc, argv, "prj_drone_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	// Publishers
	ros::Publisher chatter_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
	ros::Publisher Land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Publisher reset_pub = n.advertise<std_msgs::Empty>("/ardrone/reset", 1000);

	// Subscribers
	ros::Subscriber ar_pose_sub = n.subscribe("ar_pose_marker", 1, MarkerCallback);
	ros::Subscriber navdata_sub = n.subscribe("/ardrone/navdata", 1000, NavdataCallback);

	// Service clients
	ros::ServiceClient camaras_sol = n.serviceClient <std_srvs::Empty> ("/ardrone/togglecam");

	// Set up a timer
	ros::Timer timer = n.createTimer(ros::Duration(1), callbackTimer);

	// Set up dynamic reconfigure
	dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig> server;
	dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig>::CallbackType f;

	f = boost::bind(&dynrec_callback, _1, _2);
	server.setCallback(f);

	// Set up states
	CURRENT_STATE_ = LANDED;
	cam_ = false;
	turning_ = false;
	waiting_ = false;
	performing_ = false;
	waiting2_ = false;
	ready2takeoff_ = true;

	// Main loop
	while (ros::ok())
	{
		// Take off
		if (actions_todo.update_takeoff && ready2takeoff_){
			std_msgs::Empty msg;
			chatter_pub.publish(msg);
			ready2takeoff_=false;
			ROS_INFO("I want to take off");
		}

		// Land
		if (actions_todo.update_land){
			std_msgs::Empty msgL;
			Land_pub.publish(msgL);
		}

		// Toggle camera
		if (actions_todo.update_toggle_cam){
			tgcam(camaras_sol);
			actions_todo.update_toggle_cam=false;
		}

		// Reset ArDrone
		if (actions_todo.update_reset){
			ROS_INFO("Reset!");
			std_msgs::Empty msg;
			reset_pub.publish(msg);
			actions_todo.update_reset=false;
		}


		// State transitions
		if(CURRENT_STATE_ != RECEIVED_STATE_){
			std::string state;

			switch(RECEIVED_STATE_){
				case TAKINGOFF :
					state = "Taking off";
					if(!cam_){
						tgcam(camaras_sol);
					}
					break;
				case FLYING :
					if(CURRENT_STATE_==TAKINGOFF){
						turning_ = true;
						prev_time_ = time_;
					}
					state = "Hovering";
					break;
				case LANDING :
					state = "Landing";
					break;
				case LANDED :
					state = "Landed";
					break;
			}
			ROS_INFO("STATE: %s", state.c_str());

			CURRENT_STATE_ = RECEIVED_STATE_;
		}


		if(CURRENT_STATE_==FLYING){
			if (turning_){
				double errX, errY, errZ, errYaw, Kx=1, Ky=1, Kz=1, Kyaw=1;
				geometry_msgs::Twist msg;

				errX = desired_distance_x_-_x_;
				errY = desired_distance_y_-_y_;
				errZ = desired_distance_z_-_z_;
				errYaw = desired_angle_yaw_-yaw_;

				msg.linear.y=Kx*errX;
				msg.linear.x=Ky*errY;
				msg.linear.z=Kz*errZ;
				msg.angular.z=Kyaw*errYaw;
				twist_pub.publish(msg);

				if(ros::Time::now()-prev_time_>TIME_TURNING_){
					turning_  = false;
					waiting_ = true;
					prev_time_ = ros::Time::now();
					tgcam(camaras_sol);
					ROS_INFO("Now, I'm not turning.");
				}
			}
			if (waiting_ && ros::Time::now()-prev_time_>TIME_WAITING_){
				waiting_ = false;
				performing_ = true;
				t0_ = ros::Time::now();
				ROS_INFO("Now, I'm starting my movement.");
			}
			if (performing_){
				double errX, errY, errZ, errYaw, Kx=0.3, Ky=1, Kz=1, threshold=0.1;
				geometry_msgs::Twist msg;

				errX = final_distance_x_-_x_;
				errY = final_distance_y_-_y_;
				errZ = final_distance_z_-_z_;

				msg.linear.x=Kx*errX;
				msg.linear.y=Ky*errY;
				msg.linear.z=Kz*errZ;
				
				twist_pub.publish(msg);
				//ROS_INFO("Error: %f\n", errX);

				if(errX<threshold){
					performing_=false;
					waiting2_=true;
					prev_time_=ros::Time::now();
					ROS_INFO("Now, I have ended the execution of my movement.");
				}
			}
			if (waiting2_ && ros::Time::now()-prev_time_>TIME_WAITING2_){
				waiting2_=false;
				std_msgs::Empty msgL;
				Land_pub.publish(msgL);
				Land_pub.publish(msgL);
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}


	// Reset ArDrone when stopping node
	std_msgs::Empty msg;
	reset_pub.publish(msg);

	return 0;
}


