#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H

class  DynamicWindowApproach
{
    public:
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
        }

    private:

        Window dw;
        double dt;
        double max_speed;
        double min_speed;
        double max_yawrate;
        double max_accel;
        double max_dyawrate;
        double predict_time;
        double cost_heading;
        double cost_velocity;
        double cost_obs;

        State current_state;
        State roomba_traj;
        std::vector<State> traj;

        void local_goal_callback(const geometry_msgs::Posestamped::ConstPtr &msg);
        void pose_callback(const geometry_msgs::Posestamped::ConstPtr &msg);
        void create_dynamic_window();
        void calc_trajectory(const double &v, const double &omega);
        double calc_evalation(std::vector<State>
        double calc_cost_heading(std::vector<State>& traj);
        double calc_cost_velocity(std::vector<State>& traj);

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_local_goal;
        ros::Subscriber sub_pose;

        ros::Publisher pub_predict_path;

        nav_msgs::Path predict_path;
        geometry_msgs::Posestamped local_goal;
        geometry_msgs::Posestamped pose;
};

#endif


