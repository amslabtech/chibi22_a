#include "local_path_planner/local_path_planner.h"

Dynamic_Window_Approach::Dynamic_Window_Approach():private_nh("~")
{
    private_nh.param("max_speed",max_speed,{1.0});
    private_nh.param("min_speed",min_speed,{0.0});
    private_nh.param("max_yawrate",max_yawrate,{0.8});
    private_nh.param("max_accel",max_accel,{1.0});
    private_nh.param("max_dyawrate",max_dyawrate,{2.0});
    private_nh.param("dt",dt,{0.5});
    private_nh.param("predict_time",predict_time,{3.0});
    private_nh.param("cost_heading",cost_heading,{1.0});
    private_nh.param("cost_velocity",cost_velocity,{1.0});
    private_nh.param("cost_obs",cost_obs,{1.0});

    sub_local_goal = nh.subscribe("local_goal",10,&Dynamic_Window_Approach::local_goal_callback,this);
    sub_pose = nh.subscribe("pose",10,&Dynamic_Window_Approach::pose_callback,this);

    pub_predict_path = nh.advertise<nav_msgs::Path>("predict_path",1);
}

void Dynamic_Window_Approach::local_goal_callback(const geometry_msgs::Posestamped::ConstPtr& msg)
{
    local_goal=*msg;
}

void Dynamic_Window_Approach::pose_callback(const geometry_msgs::Posestamped::ConstPtr& msg)
{
    pose=*msg;
}

//ダイナミックウィンドウを作成
void Dynamic_Window_Approach::create_dynamic_window()
{
    std::vector<float> Vs = {max_speed,min_speed,max_yawrate,-1*max_yawrate};

    std::vector<float> Vd = {
        current_state.v + max_accel*dt,
        current_state.v - max_accel*dt,
        current_state.omega + max_dyawrate,
        current_state.omega - max_dyawrate
    };

    dw.max_v = std::min(Vs[0],Vd[0]);
    dw.min_v = std::max(Vs[1],Vd[1]);
    dw.max_yawrate = std::min(Vs[2],Vd[2]);
    dw.min_yawrate = std::max(Vs[3],Vd[3]);

}

//軌跡を計算
void Dynamic_Window_Approach::calc_trajectory(const double &v, const double &omega)
{
    traj.clear();

    predict_path.poses.clear();
    roomba_traj = {0.0,0.0,0.0,0.0,0.0};

    for(double time=0; time<=predict_time;time+=dt)
    {
        roomba_traj.yaw += omega * dt;
        roomba_traj.x += v * std::sin(roomba_traj.yaw) * dt;
        roomba_traj.y += v * std::cos(roomba_traj.yaw) * dt;
        roomba_traj.v = v;
        roomba_traj.omega = omega;
        traj.push_back(roomba_traj);

        geometry_msgs::Posestamped sim_pose;
        sim_pose.pose.position.x = roomba_traj.x;
        sim_pose.pose.position.y = roomba_traj.y;
        predict_path.poses.push_back(sim_pose);
    }
    predict_path.header.frame.id= "base_link";
    pub_predict_path.publish(predict_path);
}

//評価関数の計算
double Dynamic_Window_Approach::calc_evaluation(std::vector<State>& traj)
{
    double cost_heading = calc_cost_heading(traj) * weight_heading;
    double cost_velocity = calc_cost_velocity(traj) * weight_velocity;
    double cost_obs = calc_cost_obs(traj) * weight_obs;

    double cost_total = cost_heading + cost_velocity + cost_obs;

    return cost_total;
}

//ゴールにどれだけ向いているか
double Dynamic_Window_Approach::calc_cost_heading(std::vector<State>& traj)
{
    double goal_theta = std::atan2(local_goal.pose.position.y - pose.pose.position.y - traj.back().y,local_goal.pose.position.x - pose.pose.position.x - traj.back().x);
    double score_angle = goal_theta - tf::getYaw(pose.pose.orientation) - traj.back().yaw;
    if(score_angle >  M_PI) score_angle -= 2*M_PI;
    if(score_angle < -M_PI) score_angle += 2*M_PI;

    return std::abs(score_angle)/M_PI;
}

double Dynamic_Window_Approach::calc_cost_velocity(std::vector<State>& traj)
{
    return (max_speed - traj.back().v)/max_speed;
}

double Dynamic_Window_Approach::calc_cost_obstacle()
{
    double dist_min = 1e3;

    for(auto& state : traj){
    }
}


