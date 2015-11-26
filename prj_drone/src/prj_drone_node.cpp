#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "dynamic_reconfigure/server.h"
#include "prj_drone/dynamic_paramsConfig.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include "ar_pose/ARMarkers.h"

// Sctructure containing actions to be done and desired velocities
struct ActionsToDo
{
	bool update_takeoff;
	bool update_land; 
	bool update_cambio_camara;
	bool update_reset;
	double update_velocidadX;
	double update_velocidadY;
	double update_velocidadZ;
	double update_angularX;
	double update_angularY;
	double update_angularZ;

	ActionsToDo(){
		update_takeoff=false;
		update_land=false;
		update_cambio_camara=false;
		update_reset=false;
		update_velocidadX=false;
		update_velocidadY=false;
		update_velocidadZ=false;
		update_angularX=false;
		update_angularY=false;
		update_angularZ=false;
	}
};
ActionsToDo actions_todo; 


double marker_z_;
double marker_y_;
double marker_x_;

double distancia_deseada_z_=0; 
double distancia_deseada_y_=0; 
double distancia_deseada_x_=0; 


void dynrec_callback(prj_drone::dynamic_paramsConfig &config, uint32_t level) 
{
	// Handling dynamic reconfigure changes
	// Actions
	actions_todo.update_takeoff=config.take_off;
	config.take_off=false;
	actions_todo.update_land=config.land;
	config.land=false;
	actions_todo.update_cambio_camara=config.cambio_camara; 
	config.cambio_camara=false;
	actions_todo.update_reset=config.reset;
	config.reset=false;
	actions_todo.update_velocidadX=config.velocidadX; 
	actions_todo.update_velocidadY=config.velocidadY; 
	actions_todo.update_velocidadZ=config.velocidadZ; 
	actions_todo.update_angularX=config.angularX; 
	actions_todo.update_angularY=config.angularY; 
	actions_todo.update_angularZ=config.angularZ;

	// Desired distances
	distancia_deseada_z_=config.z_desitjada;
	distancia_deseada_y_=config.y_desitjada;
	distancia_deseada_x_=config.x_desitjada;

	ROS_INFO("Reconfigure Request. X: %f, Y: %f, Z: %f", distancia_deseada_x_, distancia_deseada_y_, distancia_deseada_z_);
}


void MarkerCallback(const ar_pose::ARMarkers::ConstPtr& mensage){
	// Handling marker data
	if  (mensage->markers.size()>0)
	{
		marker_z_=mensage->markers[0].pose.pose.position.z;
		marker_y_=mensage->markers[0].pose.pose.position.y;
		marker_x_=mensage->markers[0].pose.pose.position.x;
		double error=sqrt(pow(marker_x_-distancia_deseada_x_,2)+pow(marker_y_-distancia_deseada_y_,2)+pow(marker_z_-distancia_deseada_z_,2));

		ROS_INFO("I heard %f %f %f", marker_x_,marker_y_,marker_z_);
		ROS_INFO("Error modul:  %f", error);
	}
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

	// Subscriber (marker)
	ros::Subscriber ar_pose_sub = n.subscribe("ar_pose_marker", 1, MarkerCallback);

	// Service clients
	ros::ServiceClient camaras_sol = n.serviceClient <std_srvs::Empty> ("/ardrone/togglecam");
	ros::ServiceClient reset_sc = n.serviceClient<std_srvs::Empty>("/ardrone/reset", 1000);

	// SEt up dynamic reconfigure
	dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig> server;
	dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig>::CallbackType f;

	f = boost::bind(&dynrec_callback, _1, _2);
	server.setCallback(f);

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
		if (actions_todo.update_cambio_camara){
			std_srvs::Empty msgT;
			camaras_sol.call(msgT);
			ROS_INFO("Toggle camera!");
			actions_todo.update_cambio_camara=false;	
		}

		// Reset ArDrone
		if (actions_todo.update_reset){
			ROS_INFO("Reset!");
			std_srvs::Empty msg;
			reset_sc.call(msg);
			actions_todo.update_reset=false;
		}

		// Send Twist
		if (actions_todo.update_velocidadX || actions_todo.update_velocidadY || actions_todo.update_velocidadZ || 
			actions_todo.update_angularZ){
			geometry_msgs::Twist msgV;
			msgV.linear.x= actions_todo.update_velocidadX;
			msgV.linear.y= actions_todo.update_velocidadY;
			msgV.linear.z= actions_todo.update_velocidadZ;
			msgV.angular.z= actions_todo.update_angularZ;
			Velocidad_pub.publish(msgV);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}


	// Reset ArDrone when stopping node
	std_srvs::Empty msg;
	reset_sc.call(msg);

	return 0;
}
