#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "dynamic_reconfigure/server.h"
#include "prj_drone/dynamic_paramsConfig.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include "ar_pose/ARMarkers.h" //MARKERS

// NO CALCULA EL ERROR


struct ActionsToDo
{
   bool update_takeoff;
   bool update_land; 
   bool update_cambio_camara;
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
   update_velocidadX=false;
   update_velocidadY=false;
   update_velocidadZ=false;
   update_angularX=false;
   update_angularY=false;
   update_angularZ=false;

  }
};
ActionsToDo actions_todo; //actions_todo es una variable tipo ActionsToDo en la que guardo las órdenes del dynamic config


double marker_z_; // per a saber que és una variable global puc posar_ 
double marker_y_;
double marker_x_;

double distancia_deseada_z_=0; 
double distancia_deseada_y_=0; 
double distancia_deseada_x_=0; 







void dynrec_callback(prj_drone::dynamic_paramsConfig &config, uint32_t level) // config té totes les var del dynamic config
//void takeoffCallback(const std_msgs::Empty &msg);
{
	//ROS_INFO("New Desired position: (%f,%f,%f). Print screen: %s", config.x, config.y, config.z,	config.print_screen ? "True" : "False");
    
   actions_todo.update_takeoff=config.take_off;
	config.take_off=false;
   actions_todo.update_land=config.land;
	config.land=false;
   actions_todo.update_cambio_camara=config.cambio_camara; 
        config.cambio_camara=false;
   actions_todo.update_velocidadX=config.velocidadX; 
   actions_todo.update_velocidadY=config.velocidadY; 
   actions_todo.update_velocidadZ=config.velocidadZ; 
   actions_todo.update_angularX=config.angularX; 
   actions_todo.update_angularY=config.angularY; 
   actions_todo.update_angularZ=config.angularZ; 

// leo error distance
   distancia_deseada_z_=config.z_desitjada;
   distancia_deseada_y_=config.y_desitjada;
   distancia_deseada_x_=config.x_desitjada;

   ROS_INFO("Reconfigure Request: %f %f %f", distancia_deseada_x_, distancia_deseada_y_, distancia_deseada_z_);

	// Store new acquired values
	//desired_pose_.clear();
	//desired_pose_.push_back(config.x);
	//desired_pose_.push_back(config.y);
	//desired_pose_.push_back(config.z);
	//print_screen_ = config.print_screen;
}



void MarkerCallback(const ar_pose::ARMarkers::ConstPtr& mensage){

    if  (mensage->markers.size()>0)
    {
	marker_z_=mensage->markers[0].pose.pose.position.z;
	marker_y_=mensage->markers[0].pose.pose.position.y;
	marker_x_=mensage->markers[0].pose.pose.position.x;
        ROS_INFO("I heard %f %f %f", marker_x_,marker_y_,marker_z_);

//        double error=marker_z_-distancia_deseada_z_;
	double error=sqrt(pow(marker_x_-distancia_deseada_x_,2)+pow(marker_y_-distancia_deseada_y_,2)+pow(marker_z_-distancia_deseada_z_,2));

	ROS_INFO("Error modul:  %f", error);
    }
}



/*
void takeoffCallback(const std_msgs::Empty::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data.c_str());
  ROS_INFO("I heard you are taking off");
}

void landCallback(const std_msgs::Empty::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data.c_str());
  ROS_INFO("I heard you are landing");
}

void cmd_velCallback(const std_msgs::Empty::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data.c_str());
  ROS_INFO("I heard you are moving");
}
*/


int main(int argc, char **argv)
{
  ros::init(argc, argv, "prj_drone_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  

//  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
//    ros::Subscriber sub = n.subscribe("/ardrone/takeoff", 1000,takeoffCallback);
//    ros::Subscriber sub_1 = n.subscribe("/ardrone/land", 1000,landCallback);
//    ros::Subscriber sub_2 = n.subscribe("/cmd_vel", 1000,cmd_velCallback);
    
    ros::Publisher chatter_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);

    ros::Publisher Land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
    ros::Publisher Velocidad_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); // <package::tipo_mensaje>("topic")

    ros::ServiceClient camaras_sol = n.serviceClient <std_srvs::Empty> ("/ardrone/togglecam"); //<paquete::tipo_mensaje>("nombre del servicio")

    ros::Subscriber ar_pose_sub = n.subscribe("ar_pose_marker", 1, MarkerCallback);	//MARKER


//  ROS_INFO("-- Starting simple subscriber node");
   dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig> server;
   dynamic_reconfigure::Server<prj_drone::dynamic_paramsConfig>::CallbackType f;

   f = boost::bind(&dynrec_callback, _1, _2);
   server.setCallback(f);


  while (ros::ok())
  {
 
   if (actions_todo.update_takeoff){
      	std_msgs::Empty msg;
      	chatter_pub.publish(msg);
    }

   if (actions_todo.update_land){
      	std_msgs::Empty msgL;
      	Land_pub.publish(msgL);
    }
   if (actions_todo.update_cambio_camara){
      //std_msgs::Empty msgL;
      //Land_pub.publish(msgL);
	std_srvs::Empty msgT;
	camaras_sol.call(msgT);
      	ROS_INFO("Tocaria solicitar el servicio de cambio de camara");
	actions_todo.update_cambio_camara=false;	
    }


    if (actions_todo.update_velocidadX || actions_todo.update_velocidadY || actions_todo.update_velocidadZ || 
    	actions_todo.update_angularZ){
    	geometry_msgs::Twist msgV;
    	msgV.linear.x= actions_todo.update_velocidadX;
    	msgV.linear.y= actions_todo.update_velocidadY;
    	msgV.linear.z= actions_todo.update_velocidadZ;
    	msgV.angular.z= actions_todo.update_angularZ;
    	Velocidad_pub.publish(msgV);
    }

 

/*
geometry_msgs/Twist
geometry_msgs/Vector3 linear
  float64 

  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
*/

	
  
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
