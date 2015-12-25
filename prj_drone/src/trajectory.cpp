
#include <ros/ros.h>
#include "trajectory.h"
#include <math.h>

// COORDS
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

/*
 *Trajectory constructor from DMP information.
 *Inputs:
 *    - dmp::LearnDMPFromDemo dmp_: Service message defining the DMP.
 *    - viapoint init: Initial viapoint of the desired trajectory.
 *    - float goal[3]: Final XYZ position.
 *    - float goal_threshold[3]: Tolerance in reaching the goal.
 *    - float seg_length: length of the trajectory. (-1= Until convergence)
 *    - float tau: Duration multiplier.
 *    - float dt: Time resolution of the plan in seconds
 *    - int integrate_iter: Internal parameter. Usually 1.
 */
trajectory::trajectory(dmp::LearnDMPFromDemo dmp_, viapoint init, float goal[3],float goal_threshold[3], float seg_length, float tau, float dt, int integrate_iter, ros::NodeHandle n)
{
    // Set Active DMP
    ros::ServiceClient srv_client_set = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");

    dmp::SetActiveDMP srv_set;
    srv_set.request.dmp_list = dmp_.response.dmp_list;
     
    if(srv_client_set.call(srv_set)){
	ROS_INFO("Set new DMP active!");
    }else{
	ROS_ERROR("Could not set the DMP active.");
    }

    // Construct service message
    ros::ServiceClient srv_client = n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
    dmp::GetDMPPlan srv;

    std::vector<double> goal_, goal_threshold_, x_0d, veld;
    std::vector<float> x_0 = init.getXYZ();
    std::vector<float> x_dot_0 = init.getVelocity().getPose();
    float t_0 = init.getTime();
    for(int i=0;  i<3; i++){
	goal_.at(i) = (double)goal[i];
	goal_threshold_.at(i) = (double)goal_threshold[i];
	x_0d.at(i) = (double)x_0.at(i);
	veld.at(i) = (double)x_dot_0.at(i);
	//vel.pop_back();
    }
    srv.request.x_0 = x_0d;
    srv.request.x_dot_0 = veld;
    srv.request.t_0 = t_0;
    srv.request.goal = goal_;
    srv.request.goal_thresh = goal_threshold_;
    srv.request.seg_length = seg_length;
    srv.request.tau = tau;
    srv.request.dt = dt;
    srv.request.integrate_iter = integrate_iter;
    
    if(srv_client.call(srv)){
	ROS_INFO("I have received a trajectory from the DMP server!");
	dmp::DMPTraj dmptraj = srv.response.plan;
	
	for(int i=0; i<dmptraj.points.size(); i++){
	    dmp::DMPPoint p = dmptraj.points.at(i);
	    coords cs = coords(p.positions[0], p.positions[1], p.positions[2]);
	    coords csv = coords(p.velocities[0], p.velocities[1], p.velocities[2]);

	    viapoint vi = viapoint(dmptraj.times.at(i), cs, csv);
	}
    }else{
	ROS_ERROR("Could not get the trajectory form the DMP server.");
    }
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



