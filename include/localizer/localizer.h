#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random>
#include <tf2_ros/transform_broadcaster.h>

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
                void p_move(double dtrans, double drot1, double drot2);
            private:
                Localizer* mcl;
        };

        // var
        int hz;
        //map
        int particle_number;
        double init_x;
        double init_y;
        double init_yaw;
        double init_x_sigma;
        double init_y_sigma;
        double init_yaw_sigma;
        double intercept;
        double random_noise;
        double delta;
        double epsilon;
        bool map_get_ok = false;
        // odometry
        double move_noise_ratio;
        bool odometry_get_ok = false;
        // laser
        double laser_noise_ratio;
        int laser_step;
        double ignore_laser=0.2;
        double alpha_slow_th;
        double alpha_fast_th;
        double alpha = 0;
        double alpha_slow = alpha;
        double alpha_fast = alpha;
        double estimated_pose_w_th;
        double reset_x_sigma;
        double reset_y_sigma;
        double reset_yaw_sigma;
        double expansion_x_speed;
        double expansion_y_speed;
        double expansion_yaw_speed;
        double reset_limit;
        int reset_count = 0;
        double search_range;

        bool kld_switch = true;
        bool aug_switch = true;


        // callback
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

        // func
        // map
        double gaussian(double mu, double sigma);
        double gaussian(double mu, double sigma, double x);
        double liner(double x, double sigma);
        double w_noise(double mu, double sigma, double x);
        double chi2(double x, double k);
        int kld(double binnum);
        double y(double k);
        double adjust_yaw(double yaw);
        Particle make_particle();
        // odometry
        void kld_resampling(double dtrans, double drot1, double drot2);
        void motion_update();
        // laser
        int xy_to_map_index(double x, double y);
        double dist_from_p_to_wall(double x_start, double y_start, double yaw, double laser_range);
        double calc_w(geometry_msgs::PoseStamped &pose);
        void normalize_w();
        void estimate_pose();
        void calc_alphas();
        void augmented_resampling();
        void expansion_reset();
        void observation_update();

        void create_p_pose_array_from_p_array(std::vector<Particle> &p_array);

        // ros
        ros::NodeHandle private_nh;
        ros::NodeHandle nh;
        ros::Subscriber map_sub;
        ros::Subscriber laser_sub;
        ros::Subscriber odometry_sub;
        ros::Publisher estimated_pose_pub;
        ros::Publisher p_pose_array_pub;

        // map
        nav_msgs::OccupancyGrid map;
        std::vector<Particle> p_array;
        geometry_msgs::PoseArray p_pose_array;
        // odometry
        nav_msgs::Odometry current_odometry;
        nav_msgs::Odometry previous_odometry;
        // laser
        sensor_msgs::LaserScan laser;
        geometry_msgs::PoseStamped estimated_pose;

};
#endif
