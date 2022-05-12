#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Localizer
{
    public:
        Localizer();
        void process();
    private:
        class Particle
        {
            public:
                Particle(Localizer* localizer);
                geometry_msgs::PoseStamped p_pose;
                double w;
                void set_p(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma);
                void p_move(double dtrans, double dlot1, double dlot2);
             private:
                Localizer* mcl;
        };
        //
        // ------------------new code-----------------------------
        // geometry_msgs::PoseStamped p_pose;
        // double w;
        // void set_p(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma);
        // void p_move(double dtrans, double drot1, double drot2);
        //
        //-----------------------------------------------------------
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);

        Particle make_particle();
        double gaussian(double mu, double sigma);
        double gaussian(double mu, double sigma, double x);
        double adjust_yaw(double yaw);
        void motion_update();
        int xy_to_map_index(double x, double y);
        double dist_from_p_to_wall(double x_start, double y_start, double yaw, double laser_range);
        double calc_w(geometry_msgs::PoseStamped &pose);
        void normalize_w();
        void adaptive_resampling();
        void observation_update();
        void estimate_pose();
        void expansion_reset();
        void create_p_pose_array_from_p_array(std::vector<Particle> &p_array);

        int hz;
        int particle_number;
        double init_x;
        double init_y;
        double init_yaw;
        double init_x_sigma;
        double init_y_sigma;
        double init_yaw_sigma;
        double move_noise_ratio;
        double laser_noise_ratio;
        double search_range;
        int laser_step;
        double alpha_slow_th;
        double alpha_fast_th;
        double reset_x_sigma;
        double reset_y_sigma;
        double reset_yaw_sigma;
        double expansion_x_speed;
        double expansion_y_speed;
        double expansion_yaw_speed;
        double estimated_pose_w_th;
        double reset_limit;

        geometry_msgs::PoseStamped estimated_pose;
        double alpha = 0;
        double alpha_slow=alpha;
        double alpha_fast=alpha;

        bool map_get_ok = false;
        bool odometry_get_ok = false;
        int reset_count=0;

        ros::NodeHandle private_nh;
        ros::NodeHandle nh;
        ros::Subscriber map_sub;
        ros::Subscriber laser_sub;
        ros::Subscriber odometry_sub;
        ros::Publisher p_pose_array_pub;
        ros::Publisher estimated_pose_pub;

        nav_msgs::OccupancyGrid map;
        sensor_msgs::LaserScan laser;
        nav_msgs::Odometry current_odometry;
        nav_msgs::Odometry previous_odometry;
        // geometry_msgs::PoseStamped estimated_pose;
        geometry_msgs::PoseArray p_pose_array;
        std::vector<Particle>p_array;
};

#endif

