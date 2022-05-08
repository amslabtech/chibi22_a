#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H
#include <vector>
#include <ros/ros.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/tf.h"
#include "nav_msgs/Path.h"

class  DynamicWindowApproach
{
    public:
        DynamicWindowApproach();
        void process();

        struct Window
        {
            double max_v;
            double min_v;
            double max_yawrate;
            double min_yawrate;
        };

        struct State{
            double x;
            double y;
            double yaw;
            double v;
            double omega;
        };

    private:

        Window dw;
        double dt;
        double goal_tolerance;
        double max_speed;
        double min_speed;
        double max_yawrate;
        double max_accel;
        double max_dyawrate;
        double min_v;
        double min_yawrate;
        double predict_time;
        double gain_heading;
        double gain_velocity;
        double gain_obs;
        double wait;
        double world;
        double distance;
        double dist_min;
        double max_cost;
        double min_cost;
        double final_cost;
        double v_reso;
        double omega_reso;
        double current_velocity;
        double current_omega;
        double resolution;
        double a;
        double b;
        int safemass_x;
        int safemass_y;
        int column;
        int row;
        int hz;
        bool receive_local_map = false;

        State roomba_traj;
        std::vector<State> traj;
        std::vector<State> best_traj;
        std::vector<std::vector<int>> map;

        void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msgs);
        void obstacle_poses_callback(const geometry_msgs::PoseArray::ConstPtr &msg);
        void create_dynamic_window();
        void calc_trajectory(const double &v, const double &omega);
        void set_map();
        bool goal_reach();
        double calc_evaluation();
        double calc_cost_heading();
        double calc_cost_velocity();
        double calc_cost_obstacle();
        void visualize_traj();
        void calc_final_input();
        void roomba_control(double v,double yawrate);
        void dwa_control();

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_local_goal;
        ros::Subscriber sub_pose;
        ros::Subscriber sub_local_map;
        ros::Subscriber sub_obstacle_poses;

        ros::Publisher pub_predict_path;
        ros::Publisher pub_best_predict_path;
        ros::Publisher pub_roomba_ctrl;

        nav_msgs::Path predict_path;
        nav_msgs::Path best_predict_path;
        geometry_msgs::PoseStamped local_goal;
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseArray obstacle_poses;
        nav_msgs::OccupancyGrid local_map;
        roomba_500driver_meiji::RoombaCtrl cmd_vel;
};

#endif
