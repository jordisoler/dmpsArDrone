#ifndef __PRJ_DRONE_TRAJECTORY_H
#define __PRJ_DRONE_TRAJECTORY_H

#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "dmp/LearnDMPFromDemo.h"
#include "dmp/SetActiveDMP.h"
#include "dmp/GetDMPPlan.h"

namespace trajj{
/*
 *Class containing spatial coordinates. No extra functionalities yet.
 */
class coords
{
private:
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
public:
    coords(){};
    coords(geometry_msgs::Point p);
    coords(float x_, float y_, float z_);
    coords(float x_, float y_, float z_, float roll_, float pitch_, float yaw_);
    std::vector<float> getPose();
    float getX(){return x;}
    float getY(){return y;}
    float getZ(){return z;}
    float getYaw(){return yaw;}
    geometry_msgs::Point toMsgPoint()
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }
};

/*
 *Trajectory viapoint, contains position and velocity coordinates along with the corresponding timestamp.
 */
class viapoint
{
private:
    float ptime;
    coords position;
    coords velocity;
public:
    viapoint(float ti, coords pos, coords vel);
    float getTime(){return ptime;}
    coords getPose(){return position;}
    coords getVelocity(){return velocity;}
    void setPose(coords pose){position = pose;}
    std::vector<double> getXYZ();
    void show()
    {
        ROS_INFO("Time: %f. Position: (%f, %f, %f). Velocity: (%f, %f, %f).", ptime, position.getX(), position.getY(), position.getZ(), velocity.getX(), velocity.getY(), velocity.getZ());
    }
};

/*
 *Trajectory representation as a collection of viapoints.
 */
class trajectory
{
private:
    std::vector<viapoint> traj;
public:
    trajectory();
    trajectory(std::vector<viapoint> trajini);
    trajectory(dmp::LearnDMPFromDemo dmp_, viapoint init, double goal[3], double goal_threshold[3], double seg_length, double tau, double dt, int integrate_iter, ros::NodeHandle n);

    std::vector<float> getTimes();
    std::vector<coords> getPoses();
    std::vector<coords> getVelocities();
    std::vector<float> getX();
    std::vector<float> getY();
    std::vector<float> getZ();
    viapoint getPoint(int i);
    viapoint back(){return traj.back();}
    void addPoint(viapoint vp);
    void addPoint(viapoint vp, int i);
    void removePoint(int i);
    void removeLast();
    dmp::LearnDMPFromDemo learn(float gains[3], int nbf, ros::NodeHandle n);
    double size(){return traj.size();}
    double getInitTime()
    {
        return traj.at(0).getTime();
    }
    coords getInitPose()
    {
        return traj.at(0).getPose();
    }
    coords getInitVelocity()
    {
        return traj.at(0).getVelocity();
    }
    double duration()
    {
        return traj.back().getTime()-traj.front().getTime();
    }
    coords getPoint(double timei)
    {
        std::vector<float> times = this->getTimes();
        float incTime, frac, dx, dy, dz;
        std::vector<float> prev (3,0), post (3,0), result (3,0);
        float maxTime = *std::max_element(times.begin(), times.end());
        float minTime = *std::min_element(times.begin(), times.end());
        //ROS_INFO("Time: %f. Min: %f. Max: %f.", timei, minTime, maxTime);
        
        if (timei > maxTime){
            return traj.back().getPose();
        }
        if (timei < minTime){
            return traj.front().getPose();
        }
        //ROS_INFO("Times(0): %f.", times.at(0));
        int i = 0;
        std::vector<std::vector<float> > v (2, result);
        bool found = false;
        for (int k = 0; k<times.size(); ++k){
            //ROS_INFO("Current time = %f. Time(k) = %f.", timei, times.at(k));
            if(!found && timei < times.at(k)){
                //ROS_INFO("Current time = %f. Time(k) = %f.", timei, times.at(k));
                //ROS_INFO("Found! K=%u", k);
                prev = traj.at(k-1).getPose().getPose();
                post = traj.at(k).getPose().getPose();
                found = true;
                incTime = times.at(k)-times.at(k-1);
            }
        }

        for (int j=0; j<3; ++j){
            float fprev = prev.at(j), fpost = post.at(j);
            result.at(j) = (fpost-fprev)*frac+fprev;
        }
        return coords(result.at(0), result.at(1), result.at(2));
    }

    void setNullLastVelocity()
    {
        viapoint vi = traj.back();
        viapoint vi_new(vi.getTime(), vi.getPose(), coords(0,0,0));
        traj.back() = vi_new;
    }

    void shiftPose(coords xyz)
    {
        for(int i = 0; i<traj.size(); ++i){
            viapoint vi = traj.at(i);
            coords cs = vi.getPose();
            coords cs_new(cs.getX()-xyz.getX(), cs.getY()-xyz.getY(), cs.getZ()-xyz.getZ());
            vi.setPose(cs_new);
            traj.at(i)=vi;
        }
    }

    void root()
    {
        this->shiftPose(traj.front().getPose());
    }

    void show()
    {
        ROS_INFO("Trajectory information:");
        for (std::vector<viapoint>::iterator it = traj.begin(); it != traj.end(); ++it){
            viapoint vi = *it;
            vi.show();
        }
    }
};


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

coords::coords(geometry_msgs::Point p)
{
    x = p.x;
    y = p.y;
    z = p.z;
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

std::vector<float> coords::getPose()
{
    std::vector<float> out (3,0);
    out.at(0) = x;
    out.at(1) = y;
    out.at(2) = z;
    return out;
}


//VIAPOINT
viapoint::viapoint(float ti_, coords pos_, coords vel_)
{
    ptime = ti_;
    position = pos_;
    velocity = vel_;
}
std::vector<double> viapoint::getXYZ()
{
    std::vector<double> out (3,0);
    out.at(0)=position.getX();
    out.at(1)=position.getY();
    out.at(2)=position.getZ();
    //ROS_INFO("Position = (%f, %f,  %f)", position.getX(), position.getY(), position.getZ());
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
trajectory::trajectory(dmp::LearnDMPFromDemo dmp_, viapoint init, double goal[3], double goal_threshold[3], double seg_length, double tau, double dt, int integrate_iter, ros::NodeHandle n)
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

    std::vector<double> goal_ (3,0), goal_threshold_ (3,0), x_0d (3,0), veld (3,0);
    std::vector<double> x_0 = init.getXYZ();
    std::vector<float> x_dot_0 = init.getVelocity().getPose();
    float t_0 = init.getTime();
    for(int i=0;  i<3; i++){
        goal_.at(i) = goal[i];
        goal_threshold_.at(i) = goal_threshold[i];
        x_0d.at(i) = x_0.at(i);
        veld.at(i) = x_dot_0.at(i);
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
    dmp::DMPTraj dmptraj = srv.response.plan;
    int npoints = dmptraj.points.size();
    ROS_INFO("I have received a trajectory from the DMP server with %u points!", npoints);

    for(int i=0; i<npoints; i++){
        dmp::DMPPoint p = dmptraj.points.at(i);
        coords cs = coords(p.positions[0], p.positions[1], p.positions[2]);
        coords csv = coords(p.velocities[0], p.velocities[1], p.velocities[2]);

        viapoint vi = viapoint(dmptraj.times.at(i), cs, csv);

        this->addPoint(vi);
    }
    }else{
        ROS_ERROR("Could not get the trajectory from the DMP server.");
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

void trajectory::addPoint(viapoint vp){ 
    coords cpose = vp.getPose();
    //ROS_INFO("Add point function: Pose introduced: (%f, %f, %f)", cpose.getX(), cpose.getY(), cpose.getZ());
    traj.push_back(vp);
}

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
        pt.positions = traj.at(i).getXYZ();
        tr.points.push_back(pt);
        tr.times.push_back(times_traj.at(i));
    }

    std::vector<double> dgains (3,0);
    std::vector<double> kgains (3,0);
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

}

#endif




