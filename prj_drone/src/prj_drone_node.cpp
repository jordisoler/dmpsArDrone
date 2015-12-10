#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "dynamic_reconfigure/server.h"
#include "prj_drone/dynamic_paramsConfig.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
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

#define TIME_TURNING_ 6

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

bool cam_, turning_;	// Indicates wether the bottom cam is  active or not.

double marker_z_, marker_y_, marker_x_, yaw_;
double desired_distance_z_=1, desired_distance_y_=0, desired_distance_x_=0, desired_angle_yaw_=1.57; 

ros::Time time_, prev_time_;
// CALLBACKS
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


void MarkerCallback(const ar_pose::ARMarkers::ConstPtr& message){
	// Handling marker data
	if  (message->markers.size()>0)
	{
		double roll, pitch;
		marker_z_=message->markers[0].pose.pose.position.z;
		marker_y_=message->markers[0].pose.pose.position.y;
		marker_x_=message->markers[0].pose.pose.position.x;
		
		tf::Quaternion qt;
		tf::quaternionMsgToTF(message->markers[0].pose.pose.orientation,qt);
		tf::Matrix3x3(qt).getRPY(roll, pitch, yaw_);

		//double error=sqrt(pow(marker_x_-desired_distance_x_,2)+pow(marker_y_-desired_distance_y_,2)+pow(marker_z_-desired_distance_z_,2));

		//ROS_INFO("I heard %f %f %f", marker_x_,marker_y_,marker_z_);
		//ROS_INFO("Error modul:  %f", error);
	}
}


void NavdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg_Navdata){
	// Getting ArDrone state
	RECEIVED_STATE_ = msg_Navdata->state;
}

void callbackTimer(const ros::TimerEvent& tEvent){
	// Timer callback
	time_ = tEvent.last_real;
}


// Auxiliary functions
void tgcam(ros::ServiceClient camaras_sc){
	std_srvs::Empty msgT;
	camaras_sc.call(msgT);
	ROS_INFO("Toggle camera!");
	cam_ = !cam_;
	turning_ = turning_ && cam_;
}

// Main function
int main(int argc, char **argv){
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

	// Main loop
	while (ros::ok()){
		// Take off
		if (actions_todo.update_takeoff){
			std_msgs::Empty msg;
			chatter_pub.publish(msg);
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

		//geometry_msgs::Twist empty_T;
		//twist_pub.publish(empty_T);

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
					state = "Hovering";
					turning_ = true;
					prev_time_ = time_;
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

				errX = desired_distance_x_-marker_x_;
				errY = desired_distance_y_-marker_y_;
				errZ = desired_distance_z_-marker_z_;
				errYaw = desired_angle_yaw_-yaw_;

				msg.linear.y=Kx*errX;
				msg.linear.x=Ky*errY;
				msg.linear.z=Kz*errZ;
				msg.angular.z=Kyaw*errYaw;
				twist_pub.publish(msg);

				if(ros::Time::now()-prev_time_>ros::Duration(TIME_TURNING_)){
					turning_  = false;
					prev_time_ = ros::Time::now();
					ROS_INFO("Now, I'm not turning.");
				}
			}
		}
		

		/* Send Twist
		if (actions_todo.update_velocityX || actions_todo.update_velocityY || actions_todo.update_velocityZ || 
			actions_todo.update_angularZ){
			geometry_msgs::Twist msgV;
			msgV.linear.x= actions_todo.update_velocityX;
			msgV.linear.y= actions_todo.update_velocityY;
			msgV.linear.z= actions_todo.update_velocityZ;
			msgV.angular.z= actions_todo.update_angularZ;
			twist_pub.publish(msgV);
		}*/

		ros::spinOnce();
		loop_rate.sleep();
	}


	// Reset ArDrone when stopping node
	std_msgs::Empty msg;
	reset_pub.publish(msg);

	return 0;
}
