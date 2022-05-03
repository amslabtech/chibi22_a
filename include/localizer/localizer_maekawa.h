#ifndef LOCALIZER_MAEKAWA_H
#define LOCALIZER_MAEKAWA_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Odometry.h>
#include <random>
#include <ros/ros.h>
#include <tf/tf.h>
// #include <sensor_msgs/LaserScan.h>
class Localizer
{
    public:
        Localizer();
        void process();
    private:
        //------------------hensuu-------------------
        // class Particle
        // {
        //     public:
        //         Particle(Localizer* localizer);
        //         geometry_msgs::PoseStamped p_pose;
        //         double weight;
        //     private:
        //         Localizer* mcl;
        // };

        geometry_msgs::PoseStamped p_pose;
        int hz;
        int particle_number;
        double weight;
        double initial_x;
        double initial_y;
        double initial_yaw;
        double initial_x_sigma;
        double initial_y_sigma;
        double initial_yaw_sigma;
        // double move_noise_ratio;
        // double laser_noise_ratio;
        // double scan_range;
        // int laser_step;
        // double alpha_slow_th;
        // double alpha_fast_th;
        // double alpha=0;
        // double alpha_slow=alpha;
        // double alpha_fast=alpha;
        // double reset_x_sigma;
        // double reset_y_sigma;
        // double reset_yaw_sigma;
        // double expansion_x_speed;
        // double expansion_y_speed;
        // double expansion_yaw_speed;
        // double estimated_pose_w_th;
        // double reset_limit;
        // int reset_count=0;


        bool map_get_ok=false;
        // bool odometry_get_ok=false;

        ros::NodeHandle nh;
        ros::NodeHandle Localizer_nh;
        ros::Subscriber map_sub;
        // ros::Subscriber odometry_sub;
        ros::Publisher p_pose_array_pub;
        // ros::Publisher estimated_pose_pub;

        nav_msgs::OccupancyGrid map;
        // sensor_msgs::LaserScan laser;
        // nav_msgs::Odometry current_odometry;
        // nav_msgs::Odometry previous_odometry;

        // geometry_msgs::PoseStamped estimated_pose;
        geometry_msgs::PoseArray p_pose_array;
        // std::vector<Particle>p_array;
        std::vector<geometry_msgs::PoseStamped> p_array;



        //------------------kansuu-----------------------

                          //----callback------
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        // void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        // void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

        geometry_msgs::PoseStamped make_particle();
        void set_particle(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma);
        double gaussian(double mu, double sigma);
        double adjust_yaw(double yaw);

        void create_p_pose_array_from_p_array(std::vector<geometry_msgs::PoseStamped> &p_array);
};


#endif


















