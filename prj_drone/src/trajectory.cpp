
#include <ros/ros.h>
#include "trajectory.h"
#include <math.h>


coords::coords(float x_, float y_, float z_)
{
    x = x_;
    y = y_;
    z = z_;
    roll = 0;
    pitch = 0;
    yaw = 0;
}

coords::coords(float x_, float y_, float z_, float roll_, float pitch_, float yaw_)
{
    x = x_;
    y = y_;
    z = z_;
    roll = roll_;
    pitch = pitch_;
    yaw = yaw_;
}


//VIAPOINT
viapoint::viapoint(float ti, coords pos, coords vel)
{
    ptime = ti;
    pos = pos;
    velocity = vel;
}
std::vector<float> viapoint::getXYZ()
{
    std::vector<float> out;
    out.push_back(position.getX());
    out.push_back(position.getY());
    out.push_back(position.getZ());
    return out;
}

// TRAJECTORY
trajectory::trajectory(){}

trajectory::trajectory(std::vector<viapoint> trajini)
{
    traj = trajini;
}

std::vector<float> trajectory::getTimes()
{
    std::vector<float> out;
    for(int i=0; i<traj.size(); i++) out.push_back(traj.at(i).getTime());

    return out;
}
std::vector<coords> trajectory::getPoses()
{
    std::vector<coords> out;
    for(int i=0; i<traj.size(); i++) out.push_back(traj.at(i).getPose());

    return out;
}
std::vector<coords> trajectory::getVelocities()
{
    std::vector<coords> out;
    for(int i=0; i<traj.size(); i++) out.push_back(traj.at(i).getVelocity());

    return out;
}

std::vector<float> trajectory::getX()
{
    std::vector<float> out;
    for(int i=0; i<traj.size(); i++) out.push_back(traj.at(i).getPose().getX());

    return out;
}
std::vector<float> trajectory::getY()
{
    std::vector<float> out;
    for(int i=0; i<traj.size(); i++) out.push_back(traj.at(i).getPose().getY());

    return out;
}
std::vector<float> trajectory::getZ()
{
    std::vector<float> out;
    for(int i=0; i<traj.size(); i++) out.push_back(traj.at(i).getPose().getZ());

    return out;
}


viapoint trajectory::getPoint(int i){ return traj.at(i);}

void trajectory::addPoint(viapoint vp){ traj.push_back(vp);}

void trajectory::addPoint(viapoint vp, int i){ traj.at(i)=vp;}

void trajectory::removePoint(int i){ traj.erase(traj.begin()+i);}

void trajectory::removeLast(){ traj.pop_back();}

dmp::LearnDMPFromDemo trajectory::learn(float gains[], int nbf, ros::NodeHandle n)
{
    ros::ServiceClient srv_client = n.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");

    dmp::LearnDMPFromDemo srv;

    dmp::DMPTraj tr;
    std::vector<float> times_traj =  this->getTimes();
    for(int i=0; i<traj.size(); i++){
	dmp::DMPPoint pt;
	float ptf [3];
	std::vector<float> xyz = traj.at(i).getXYZ();
	for(int j=0; j<3; j++){
	    pt.positions[j] = xyz[j];
	}
	tr.points[i] = pt;
	tr.times[i] = times_traj.at(i);
    }

    std::vector<double> dgains;
    std::vector<double> kgains;
    for(int i=0; i<3; i++){
    	kgains.at(i)=gains[i];
	dgains.at(i)=2*sqrt(gains[i]);
    }

    srv.request.demo = tr;
    srv.request.k_gains = kgains;
    srv.request.d_gains = dgains;
    srv.request.num_bases = nbf;

    if(srv_client.call(srv)){
	ROS_INFO("DMP received from server!");
    }else{
	ROS_ERROR("Could not get the DMP from the server.");
    }
    return srv;
}


