#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "dynamic_reconfigure/server.h"
#include "prj_drone/dynamic_paramsConfig.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include <cstdlib>
#include "ar_pose/ARMarkers.h"


// States of the ArDrone
#define LANDED 0
#define TAKINGOFF 1
#define UP 2
#define CONTROLLING1 3
#define WAITING1 4
#define CONTROLLING2 5
#define WAITING2 6
#define GO_DOWN 7
#define LANDING 8

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

double marker_z_, marker_y_, marker_x_;
double desired_distance_z_=0, desired_distance_y_=0, desired_distance_x_=0; 

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
		marker_z_=message->markers[0].pose.pose.position.z;
		marker_y_=message->markers[0].pose.pose.position.y;
		marker_x_=message->markers[0].pose.pose.position.x;
		double error=sqrt(pow(marker_x_-desired_distance_x_,2)+pow(marker_y_-desired_distance_y_,2)+pow(marker_z_-desired_distance_z_,2));

		ROS_INFO("I heard %f %f %f", marker_x_,marker_y_,marker_z_);
		ROS_INFO("Error modul:  %f", error);
	}
}


void NavdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg_Navdata){
	// Getting ArDrone state
	RECEIVED_STATE_ = msg_Navdata->state;
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
	ros::Publisher Velocidad_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	// Subscribers
	ros::Subscriber ar_pose_sub = n.subscribe("ar_pose_marker", 1, MarkerCallback);
	ros::Subscriber navdata_sub = n.subscribe("/ardrone/navdata", 1000, NavdataCallback);

	// Service clients
	ros::ServiceClient camaras_sol = n.serviceClient <std_srvs::Empty> ("/ardrone/togglecam");
	ros::ServiceClient reset_sc = n.serviceClient<std_srvs::Empty>("/ardrone/reset", 1000);

	// Set up dynamic reconfigure
	dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig> server;
	dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig>::CallbackType f;

	f = boost::bind(&dynrec_callback, _1, _2);
	server.setCallback(f);

	// Set up states
	CURRENT_STATE_ = LANDED;

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
			std_srvs::Empty msgT;
			camaras_sol.call(msgT);
			ROS_INFO("Toggle camera!");
			actions_todo.update_toggle_cam=false;	
		}

		// Reset ArDrone
		if (actions_todo.update_reset){
			ROS_INFO("Reset!");
			std_srvs::Empty msg;
			reset_sc.call(msg);
			actions_todo.update_reset=false;
		}

		// State transitions
		if(CURRENT_STATE_ != RECEIVED_STATE_){
			// ROS_INFO("CURRENT_STATE_ = %u, RECEIVED_STATE_ = %u", CURRENT_STATE_, RECEIVED_STATE_);
			if (RECEIVED_STATE_==6){
				ROS_INFO("****************State: Taking off.**************");
				actions_todo.update_toggle_cam=true;
			}

			CURRENT_STATE_ = RECEIVED_STATE_;
		}
		

		


		/*/ Send Twist
		if (actions_todo.update_velocityX || actions_todo.update_velocityY || actions_todo.update_velocityZ || 
			actions_todo.update_angularZ){
			geometry_msgs::Twist msgV;
			msgV.linear.x= actions_todo.update_velocityX;
			msgV.linear.y= actions_todo.update_velocityY;
			msgV.linear.z= actions_todo.update_velocityZ;
			msgV.angular.z= actions_todo.update_angularZ;
			Velocidad_pub.publish(msgV);
		}*/

		ros::spinOnce();
		loop_rate.sleep();
	}


	// Reset ArDrone when stopping node
	std_srvs::Empty msg;
	reset_sc.call(msg);

	return 0;
}
