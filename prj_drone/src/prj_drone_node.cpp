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
#include <trajectory.h>
#include <csv_parser.hpp>

#include <fstream>
#include <string.h>
#include <string>
#include <iostream>
#include <cstdlib>


using namespace trajj;

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
const static ros::Duration TIME_WAITING_ = ros::Duration(1);
const static ros::Duration TIME_WAITING2_ = ros::Duration(2);

const std::string logfile = "/home/jordi/catkin_ws/src/dmpsArdrone/csv/projectData.csv";
const std::string posfile = "/home/jordi/catkin_ws/src/dmpsArdrone/csv/trajectoryData.csv";

// Trajectory CSV input
const std::string STRAIGHT_LINE = "/home/jordi/catkin_ws/src/dmpsArdrone/csv/demos/straightLine.csv";
const std::string LETTER_A = "/home/jordi/catkin_ws/src/dmpsArdrone/csv/demos/a.csv";
const std::string referencefile = LETTER_A;


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

//double _z_, _y_, _x_, yaw_;
double pose_ [3];
double desired_distance [] = {0, 0, 1};
double desired_angle_yaw_ = 1.57, yaw_;
double final_distance [] = {2, 0, 1};

ros::Time time_, prev_time_, t0_, t_start;

/**************************************************
*	AUXILLIARY FUNCTIONS
**************************************************/
/*
 *Toggle the camera taking into account the flags cam_ and turning_
 */
void tgcam(ros::ServiceClient camaras_sc)
{
	std_srvs::Empty msgT;
	camaras_sc.call(msgT);
	ROS_INFO("Toggle camera!");
	cam_ = !cam_;
	turning_ = turning_ && cam_;
}

void writeData(std::vector<double> in, std::string fileName)
{
	// Saving Data
	std::ofstream fileWriter;
	fileWriter.open(fileName.c_str(),  std::ios::app);
	if (fileWriter.fail()){
		ROS_ERROR("The file could not be opened.");
	}else{
		for(std::vector<double>::iterator it = in.begin(); it!=in.end(); ++it){
			double data = *it;
			fileWriter<<data;
			if(it+1 != in.end()){
				fileWriter<<';';
			}
		}
		fileWriter<<'\n';
		fileWriter.close();
	}
}

void logData(std::string fileName)
{
		// Logging position data
		std::vector<double> data (4,0);
		data.at(0) = ros::Time::now().toSec();
		for (int i=0; i<3; ++i){
			data.at(i+1)=pose_[i];
		}
		writeData(data, fileName);
}

/*
 *Get current pose from the markers (assuming miniproject_world)
 */
void inverseKinematics(double mx, double my, double mz)
{
    if(cam_){
		pose_[1] = mx;
		pose_[0] = my;
		pose_[2] = mz;
    }else{
		pose_[0] = 3-mz;
		pose_[1] = mx;
		pose_[2] = 1+my;
		ROS_INFO("Pose: x=%f, y=%f, z=%f", pose_[0],pose_[1],pose_[2]);
		//ROS_INFO("Pose: x=%f, y=%f, z=%f", mx,my,mz);
    }
}

/*
 *Proportional controller, guiven the desired goal, send the control action to reach it.
 */
double Pcontrol(double goal[], double gains[], int numel, ros::Publisher twist_pub)
{
    double err [numel];
    double sqrErr=0;
    geometry_msgs::Twist msg;

    for(int i=0; i<numel; i++){
    	if(i<3){
	    err[i]=goal[i]-pose_[i];
	}else{
	    err[i]=goal[i]-yaw_;
	}
	sqrErr=sqrErr+pow(err[i],2);
    }

    msg.linear.x=gains[0]*err[0];
    msg.linear.y=gains[1]*err[1];
    msg.linear.z=gains[2]*err[2];

    if(numel==4){
    	msg.angular.z=gains[3]*err[3];
    }

    twist_pub.publish(msg);
    return sqrt(sqrErr);
}

/*
 *Trajectory via point generator (straight line)
 */
geometry_msgs::Point linTraj(double trajtime, double initPose[3], double finalPose[3], ros::Duration du)
{
	double currentPose[3];
    double dn = du.toSec();
    geometry_msgs::Point pr;

    for(int i=0; i<3; i++){
    	if(dn<trajtime){
	    	currentPose[i] = (finalPose[i]-initPose[i])*dn/trajtime+initPose[i];
		}else{
			currentPose[i] = finalPose[i];
		}
    }

    pr.x = currentPose[0];
    pr.y = currentPose[1];
    pr.z = currentPose[2];
    return pr;
}

trajectory getTrajectoryFromFile(std::string file)
{
	trajectory traj;

	// CSV reading
	csv_parser csv(file.c_str());

    int nlines = csv.nlines();
    double initTime = atof(csv.get_value(1,1).c_str());
    double prevTime = initTime;
    double prevx = atof(csv.get_value(1,2).c_str());
    double prevy = atof(csv.get_value(1,3).c_str());
    double prevz = atof(csv.get_value(1,4).c_str());
    double ttime, tx, ty, tz, vx, vy,  vz, pvx, pvy, pvz, tinc;
    double vs [3], prevvs [3] = {0, 0, 0};

    ROS_INFO("Reading trajectory from file.");
    for (int i=1; i<nlines; ++i){
    	ttime = atof(csv.get_value(i,1).c_str())-initTime;
    	tx = atof(csv.get_value(i,2).c_str());
    	ty = atof(csv.get_value(i,3).c_str());

    	std::string tzs = csv.get_value(i,4);
    	tz = atof(tzs.c_str());
    	if (tz>10){
    		std::reverse( tzs.begin(), tzs.end() );
    		tz = atof(tzs.c_str());
    	}
    	//ROS_INFO("Z value (string): %s, (double): %f", csv.get_value(i,4).c_str(), tz);

    	tinc = ttime-prevTime;

    	coords ci(tx, ty, tz);

    	vs[0] = (tx-prevx)/tinc;
    	vs[1] = (ty-prevy)/tinc;
    	vs[2] = (tz-prevz)/tinc;

    	for(int i=0; i<3; ++i){
    		if(vs[i] > 1 || vs[i] < -1){
    			vs[i] = prevvs[i];
    		}
    		prevvs[i] = vs[i];
    	}

    	
    	coords veli(vs[0], vs[1], vs[2]);

    	viapoint vi(ttime, ci, veli);

    	traj.addPoint(vi);

    	prevx = tx;
    	prevy = ty;
    	prevz = tz;
    	prevTime = ttime;
    }
    traj.setNullLastVelocity();
    traj.shiftPose(coords(0, 1, 0.5));
    ROS_INFO("Trajectory successfully loaded!");
    return traj;
}

trajectory getStraightTraj(double initPose[3], double finalPose[3], double nsteps, ros::Duration du, ros::Time initTime)
{
    coords iniCoords = coords(initPose[0], initPose[1], initPose[2]);
    coords iniVel = coords(0,0,0);
    viapoint iniVp = viapoint(initTime.toSec(),iniCoords,iniVel);
    std::vector<viapoint> vp (1, iniVp);
    
    trajectory traj=trajectory(vp);
    double dus = du.toSec();
    for(int i=1; i<nsteps; i=i+1){
    	ros::Duration currentTime = ros::Duration(i*dus/nsteps);
    	geometry_msgs::Point pi = linTraj(dus, initPose, finalPose, currentTime);
    	coords ci = coords(pi);
    	coords veli = coords(0,0,0);
    	ros::Time timeAsTime = initTime+currentTime;
    	float timeInSeconds = timeAsTime.toSec();
    	viapoint vi = viapoint(timeInSeconds, ci, veli);
    	
    	ROS_INFO("Send point to trajectory. Time: %f, Point: (%f, %f, %f), Velocity: (%f, %f, %f)",timeInSeconds, ci.getX(), ci.getY(), ci.getZ(), veli.getX(), veli.getY(),  veli.getZ() );
    	traj.addPoint(vi);
    }
    return traj;
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
	desired_distance[0]=config.x_desitjada;
	desired_distance[1]=config.y_desitjada;
	desired_distance[2]=config.z_desitjada;

	ROS_INFO("Reconfigure Request. X: %f, Y: %f, Z: %f", desired_distance[0], desired_distance[1], desired_distance[2]);
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
	// Delate log files
	std::ofstream ofs;
	ofs.open(logfile.c_str(), std::ofstream::out | std::ofstream::trunc);
	ofs.close();
	ofs.open(posfile.c_str(), std::ofstream::out | std::ofstream::trunc);
	ofs.close();

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

	// Get deomnstration trajectory (Theoretical)
	/*double initPose[3] = {0, 0, 1};
	double finalPose[3] = {2, 0, 1};
	ros::Time initTime = ros::Time(ros::Time(3.0));
	ros::Duration tElapsed = ros::Duration(3.0);
	trajectory tr = getStraightTraj(initPose, finalPose, 11, tElapsed, initTime);//*/

	// Get trajecoty from a file
	trajectory tr = getTrajectoryFromFile(referencefile);
	ROS_INFO("-------------------------------Initial trajectory:---------------------------------");
	tr.show();

	// Get DMP
	float gains[3] = {1000, 1000, 1000};
	int nbf = 300;
	dmp::LearnDMPFromDemo dmpTraj = tr.learn(gains, nbf, n);

	// Get resultant trajectory
	viapoint initVp = viapoint(tr.getInitTime(), tr.getInitPose(), tr.getInitVelocity());
	//double goal[3] = {2, 0, 1};
	coords dcoords = tr.back().getPose();
	double goal[3] = {dcoords.getX(), dcoords.getY(), dcoords.getZ()};
	double gtolerance[3] = {0.1, 0.1, 0.1};
	trajectory tr2 = trajectory(dmpTraj, initVp, goal, gtolerance, -1, dmpTraj.response.tau/2, tr.duration()/50, 1, n);
	ROS_INFO("-------------------------------Trajectory to perform:---------------------------------");
	tr2.show();
	//tr2 = tr;

	// Set up states
	CURRENT_STATE_ = LANDED;
	cam_ = false;
	turning_ = false;
	waiting_ = false;
	performing_ = false;
	waiting2_ = false;
	ready2takeoff_ = true;
	t_start = ros::Time::now();

	
	// Main loop
	while (ros::ok())
	{
		logData(logfile);

	    // Take off
	    if ((ros::Time::now()-t_start>ros::Duration(10) || actions_todo.update_takeoff) && ready2takeoff_){
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
		    double Ks []={1,1,1,1};
		    double gs [4];
		    for(int i=0; i<3; i++){
			gs[i]=desired_distance[i];
		    }
		    gs[3] = desired_angle_yaw_;
		    double err = Pcontrol(gs, Ks, 4, twist_pub);

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
		if(waiting_){
			double Ks []={1,1,1,1};
		    double gs [3];
		    coords initPose = tr2.getInitPose();
		    gs[1] = initPose.getX();
   		    gs[2] = initPose.getY();
		    gs[3] = initPose.getZ();
		    ROS_INFO("Going to initial point (%f, %f, %f)", gs[0], gs[1], gs[2]);
		    double err = Pcontrol(gs, Ks, 3, twist_pub);
		}
		if (performing_){
		    double Ks [] = {0.5, 1.2, 1.2};
		    double xyz [3];
		    double threshold=0.05, err;

		    coords pcoords = tr2.getPoint((ros::Time::now()-t0_).toSec());
		    ROS_INFO("Going to point (%f, %f, %f)", pcoords.getX(), pcoords.getY(), pcoords.getZ());

		    xyz[0] = pcoords.getX();
		    xyz[1] = pcoords.getY();
		    xyz[2] = pcoords.getZ();

		    err = Pcontrol(xyz, Ks, 3, twist_pub);
		    logData(posfile);

		    if(err<threshold){
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


