#ifndef __PRJ_DRONE_TRAJECTORY_H
#define __PRJ_DRONE_TRAJECTORY_H

#include <ros/ros.h>
#include "dmp/LearnDMPFromDemo.h"

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
    coords(float x_, float y_, float z_);
    coords(float x_, float y_, float z_, float roll_, float pitch_, float yaw_);
    std::vector<float> getPose();
    float getX(){return x;}
    float getY(){return y;}
    float getZ(){return z;}
    float getYaw(){return yaw;}
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
    std::vector<float> getXYZ();
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

    std::vector<float> getTimes();
    std::vector<coords> getPoses();
    std::vector<coords> getVelocities();
    std::vector<float> getX();
    std::vector<float> getY();
    std::vector<float> getZ();
    viapoint getPoint(int i);
    void addPoint(viapoint vp);
    void addPoint(viapoint vp, int i);
    void removePoint(int i);
    void removeLast();
    dmp::LearnDMPFromDemo learn(float gains[3], int nbf, ros::NodeHandle n);
};


#endif



