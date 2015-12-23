
#include <ros/ros.h>
#include "trajectory.h"


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

