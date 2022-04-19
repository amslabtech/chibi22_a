#ifndef LOCAL_MAP_CREATOR
#define LOCAL_MAP_CREATOR

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


class Local_map_creator
{
    public:
        Local_map_creator();
        void process();
    private:
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void create_line(double yaw, double laser_range);
        void create_local_map();
        void init_map();
        int xy_to_map_index(double x, double y);
        bool check_map_range(double x, double y);
        bool is_ignore_angle(double angle);

        int hz;
        int map_size;
        int size;
        double map_reso;
        double laser_density;
        double roomba_radius;
        double ignore_angle_mergin;

        bool laser_get_check = false;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_laser;
        ros::Publisher pub_local_map;
        ros::Publisher pub_obstacle_poses;

        sensor_msgs::LaserScan laser;
        nav_msgs::OccupancyGrid local_map;
        geometry_msgs::PoseArray obstacle_poses;

};

#endif