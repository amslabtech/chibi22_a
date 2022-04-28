#ifndef LOCALIZER_MAEKAWA_H
#define LOCALIZER_MAEKAWA_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <ros/ros.h>
#include <tf/tf.h>
#include <seosor_msgs/LaserScan.h>
class Localizer
{
    public:
        Localizer();
        void process();
    private:
        //Localizer Classで用いる変数

        int hz=10;
        int particle_num=300;
        double init_x;
        double init_y;
        double init_yaw;

        double init_x_sigma;
        double init_y_sigma;
        double init_yaw_sigma;

        bool map_get_ok = false;
        bool odometry_get_ok = false;

        geometry_msgs::PoseStamped estimated_pose;
        geometry_msgs::PoseArray p_pose_array;
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber map_sub;
        ros::Publisher p_pose_array_pub;
        ros::Publisher estimated_pose_pub;

        nav_msgs::OccupancyGrid map;

};

class Particle
{
    public:
        particle();
        geometry_msgs::PoseStamped p_pose_message;
        double w;

    private:
        void make_particle()
};

#endif


















