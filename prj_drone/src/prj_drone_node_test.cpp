#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "dynamic_reconfigure/server.h"
#include "prj_drone/dynamic_paramsConfig.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include "ar_pose/ARMarkers.h" //MARKERS
#include "ardrone_autonomy/Navdata.h"
#include "tf/transform_listener.h"

#include <stdio.h>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>

/* States of the machine */
#define ABAJO 0
#define SUBIENDO 1
#define ARRIBA 2
#define CONTROLANDO1 3
#define ESPERANDO1 4
#define CONTROLANDO2 5
#define ESPERANDO2 6
#define GO_DOWN 7
#define BAJANDO 8

struct ActionsToDo
{
   bool update_takeoff;
   bool update_land; 

   ActionsToDo(){
   update_takeoff=false;
   update_land=false;
  }
};
ActionsToDo actions_todo; //actions_todo es una variable tipo ActionsToDo en la que guardo las órdenes del dynamic config


/* Definition of the variables
STATE_ = 2 -> ABAJO
STATE_ = 3 -> ARRIBA */

int STATE_;

double marker_z_; 
double marker_y_;
double marker_x_;
double marker_orient_x;
double marker_orient_y;
double marker_orient_z;
double marker_orient_w;
double desired_distance_z_=1; 
double desired_distance_y_=0; 
double desired_distance_x_=0; 
double desired_distance_z2_=1; 
double desired_distance_y2_=0; 
double desired_distance_x2_=0; 
double angulo_deseado_yaw_=1.57;

double kpZ = 1;
double kpY = 1;
double kpX = 1;
double kpZ2 = 0.3;
double kpY2 = 1;
double kpX2 = 1;
double kpYaw = 1;

double control_actionZ;
double control_actionX;
double control_actionY;
double control_actionYaw;
double errorZ = 0;
double errorX = 0;
double errorY = 0;
double errorZ2 = 0;
double errorX2 = 0;
double errorY2 = 0;
double errorYaw = 0;
double roll;
double pitch;
double yaw;
double tiempo;

bool enable_control = true;
bool enable_frontcam = true;	
bool check_time = true;


/* Definition of the functions */

void dynrec_callback(prj_drone::dynamic_paramsConfig &config, uint32_t level) 
{
	actions_todo.update_takeoff=config.take_off;
	actions_todo.update_land=config.land;
}


void MarkerCallback(const ar_pose::ARMarkers::ConstPtr& mensage){
    if  (mensage->markers.size()>0) {

		marker_z_=mensage->markers[0].pose.pose.position.z;
		marker_y_=mensage->markers[0].pose.pose.position.y;
		marker_x_=mensage->markers[0].pose.pose.position.x;

		//enable_control = true;
    	//ROS_INFO("I heard %f %f %f", marker_x_,marker_y_,marker_z_);

		// Convert quatenion to RPY
		tf::Quaternion qt;
		tf::quaternionMsgToTF(mensage->markers[0].pose.pose.orientation,qt);
		tf::Matrix3x3(qt).getRPY(roll,pitch,yaw);

		//ROS_INFO("Roll Pitch Yaw:  %f    %f    %f \n\n", roll, pitch, yaw);

		if (std::isnan(roll) || !std::isfinite(roll)|| (mensage->markers[0].pose.pose.position.z < 0)){
			ROS_WARN("Cuidadoooo, he detectado el marker por encima mio");
			enable_control=false;
		}
    }
}

void NavdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg_Navdata){
	STATE_ = msg_Navdata->state;
	//ROS_INFO("State = %d \n", STATE_);
}

void callbackTimer(const ros::TimerEvent& tEvent){
	tiempo = tEvent.last_real.toSec();
}

/*
void Control1(){
	//if( enable_control == true){ 
	geometry_msgs::Twist msgV;
   		
   	errorZ = marker_z_ - desired_distance_z_;
	errorY = marker_x_ - desired_distance_x_;
	errorX = marker_y_ - desired_distance_y_;
	//ROS_INFO("Error  X  Y  Z:  %f  %f %f \n", errorX, errorY, errorZ);
	errorYaw = angulo_deseado_yaw_ - yaw;
	//ROS_INFO("Yaw leido:  %f  , yaw deseado: %f  , control yaw: %f",yaw, angulo_deseado_yaw_, errorYaw);
		 	
	// Control X
	control_actionX = kpX* (-errorX);
	msgV.linear.x= control_actionX;

	// Control Y
	control_actionY = kpY * (-errorY);
	msgV.linear.y= control_actionY;

	// Control Z, le cambiamos el signo al error para simular la transformacion
	control_actionZ = kpZ * (-errorZ);
	msgV.linear.z= control_actionZ;
		
	// Control Yaw, el orden del error es debido a que estamos leyendo sobre el frame del marker
   	control_actionYaw = kpYaw * (errorYaw);
   	msgV.angular.z= control_actionYaw;
  // }
}	
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "prj_drone_node_test");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
    
    ros::Publisher chatter_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);

    ros::Publisher Land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
    ros::Publisher Velocidad_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); // <package::tipo_mensaje>("topic")

    ros::ServiceClient camaras_sol = n.serviceClient <std_srvs::Empty> ("/ardrone/togglecam"); //<paquete::tipo_mensaje>("nombre del servicio")
    
    ros::Subscriber ar_pose_sub = n.subscribe("ar_pose_marker", 1000, MarkerCallback);	//MARKER
	
	ros::Subscriber navdata_sub = n.subscribe("/ardrone/navdata", 1000, NavdataCallback);	//Navdata, las lecturas las envia a la funcion NavdataCallback

	ros::Timer timer = n.createTimer(ros::Duration(1), callbackTimer);

	dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig> server;
	dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig>::CallbackType f;

	f = boost::bind(&dynrec_callback, _1, _2);
   	server.setCallback(f);


   	int ESTADO = ABAJO; //ESTADO INICIAL
   	ROS_INFO("ESTADO = %d", ESTADO);

  	while (ros::ok()){

		geometry_msgs::Twist msgV;

   		if (actions_todo.update_takeoff){
	      	std_msgs::Empty msg;
    	  	chatter_pub.publish(msg);
    	}
 		if (actions_todo.update_land){
	      	std_msgs::Empty msgL;
    	  	Land_pub.publish(msgL);
	   	}

   		Velocidad_pub.publish(msgV); // Aqui publicamos toda la estructura entera dentro de la cual estan: x y z roll pitch yaw.
   	
   		/*******************************************************************/
  		/*************************** PETRI NET *****************************/
   		/*******************************************************************/
   		if(ESTADO == ABAJO && STATE_ == 6){ //Transition, STATE_=6 es take off
   			ESTADO = SUBIENDO;
   			ROS_INFO("ESTADO = %d", ESTADO);
   			std_srvs::Empty msgT;
			camaras_sol.call(msgT);
   		}
   		else if(ESTADO == SUBIENDO && STATE_== 3){ //Transition, STATE_=6 es arriba
   			ESTADO = ARRIBA;
   			ROS_INFO("ESTADO = %d", ESTADO);
   			enable_control = true;
   		}
   		else if (ESTADO == ARRIBA && enable_control == true){ //Transition
   			ESTADO = CONTROLANDO1;
   			ROS_INFO("ESTADO = %d", ESTADO);
   		}
   		else if(ESTADO == GO_DOWN && STATE_ == 8) { //Transition
   			ESTADO = BAJANDO;
   			ROS_INFO("ESTADO = %d", ESTADO);
   		}
   		else if(ESTADO == BAJANDO && STATE_ == 2){ //Transition
   			enable_control = false;
   			ESTADO = ABAJO;
   			ROS_INFO("ESTADO = %d", ESTADO);
   		}
   		else if (ESTADO == CONTROLANDO1){ //State

			errorZ = marker_z_ - desired_distance_z_;
			errorY = marker_x_ - desired_distance_x_;
			errorX = marker_y_ - desired_distance_y_;
			errorYaw = angulo_deseado_yaw_ - yaw;

			// Control X
			control_actionX = kpX* (-errorX);
			msgV.linear.x= control_actionX;
			// Control Y
			control_actionY = kpY * (-errorY);
			msgV.linear.y= control_actionY;
			// Control Z, le cambiamos el signo al error para simular la transformacion
			control_actionZ = kpZ * (-errorZ);
			msgV.linear.z= control_actionZ;
			// Control Yaw, el orden del error es debido a que estamos leyendo sobre el frame del marker
			control_actionYaw = kpYaw * (errorYaw);
			msgV.angular.z= control_actionYaw;

			Velocidad_pub.publish(msgV);

			
			if(errorYaw < 0.05){				// Tansition
   				ESTADO = ESPERANDO1;
   				ROS_INFO("ESTADO = %d", ESTADO);
   			}
   		}
   		else if(ESTADO == ESPERANDO1){ //State
            float t1;
            if (check_time){
	            ros::Time t = ros::Time::now();
	            t1 = t.toSec();
				check_time = false;
			}		

			ros::Time tt = ros::Time::now();
			float t2 = tt.toSec();
			float time = t2 - t1;
			//ROS_INFO("t1: %f   t2: %f   time:%f ",t1, t2, time); 

			if(time > 5.0){				// Tansition
			  	std_srvs::Empty msgT;   // Change the camera
				camaras_sol.call(msgT);
				sleep(1);
				check_time = true;
				ESTADO = CONTROLANDO2;
   				ROS_INFO("ESTADO = %d", ESTADO);
			}
			else{
				errorZ = marker_z_ - desired_distance_z_;
				errorY = marker_x_ - desired_distance_x_;
				errorX = marker_y_ - desired_distance_y_;
				errorYaw = angulo_deseado_yaw_ - yaw;
				// Control X
				control_actionX = kpX* (-errorX);
				msgV.linear.x= control_actionX;
				// Control Y
				control_actionY = kpY * (-errorY);
				msgV.linear.y= control_actionY;
				// Control Z, le cambiamos el signo al error para simular la transformacion
				control_actionZ = kpZ * (-errorZ);
				msgV.linear.z= control_actionZ;
				// Control Yaw, el orden del error es debido a que estamos leyendo sobre el frame del marker
   				control_actionYaw = kpYaw * (errorYaw);
   				msgV.angular.z= control_actionYaw;

   				Velocidad_pub.publish(msgV);
			}	
   		}    
   		else if(ESTADO == CONTROLANDO2){ //State

   			errorZ2 = marker_z_ - desired_distance_z2_;
			errorX2 = marker_x_ - desired_distance_x2_;
			errorY2 = marker_y_ - desired_distance_y2_;
			ROS_INFO("marker_z = %f    errorZ2 = %f", marker_z_, errorZ2);

			control_actionX = kpZ2 * (errorZ2);
			msgV.linear.x = control_actionX;

			control_actionY = kpX2 * (-errorX2);
			msgV.linear.y= control_actionY;

  			control_actionZ = kpY2 * (-errorY2);
			msgV.linear.z= control_actionZ; 

			Velocidad_pub.publish(msgV);

			if((errorZ2 < 0.06) && (errorY2 < 0.02) && (errorX2 < 0.02)) {
   				ESTADO = ESPERANDO2;
   				ROS_INFO("ESTADO = %d", ESTADO);
   			}
   		}
   		else if(ESTADO == ESPERANDO2){ //State
   			float t3;
            if (check_time){
	            ros::Time t = ros::Time::now();
	            t3 = t.toSec();
				check_time = false;
			}		

			ros::Time tt = ros::Time::now();
			float t4 = tt.toSec();
			float time2 = t4 - t3;
			ROS_INFO("t3: %f   t4: %f   time2:%f ",t3, t4, time2); 

			if(time2 > 5.0){				
				ESTADO = GO_DOWN; 
   				ROS_INFO("ESTADO = %d", ESTADO);
			}
			else{
   				errorZ2 = marker_z_ - desired_distance_z2_;
				errorX2 = marker_x_ - desired_distance_x2_;
				errorY2 = marker_y_ - desired_distance_y2_;
			
				control_actionX = kpZ2 * (errorZ2);
				msgV.linear.x = control_actionX;

				control_actionY = kpX2 * (-errorX2); 
				msgV.linear.y= control_actionY;

  				control_actionZ = kpY2 * (-errorY2);
				msgV.linear.z= control_actionZ; 

				Velocidad_pub.publish(msgV);		
			}
   		}
   		else if(ESTADO == GO_DOWN) { //State
   			std_msgs::Empty msgL; 
    	  	Land_pub.publish(msgL);
   		} 
	
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
  	return 0;
}
// El eje X de la camara delantera y el marker estan alineados y en el mismo sentido, pero para minimizar el 
// error en este eje X hay que corregir sobre el eje Y de del frame del quadcopter. Con el resto de ejes ocurre
// lo mismo.
